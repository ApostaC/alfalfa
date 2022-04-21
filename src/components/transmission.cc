#include "timestamp.hh"
#include "transmission.hh"

//TODO: Pacer is in milliseconds, so the MAX rate will be limited to MTU * 1000 byte per second!
using namespace std;
using namespace PollerShortNames;

BudgetPacer::BudgetPacer(int max_budget_)
  : max_budget_(max_budget_)
{
}

void BudgetPacer::update_budget(uint32_t now_ms)
{
  if (last_update_timestamp_ms_ == 0) {
    throw std::runtime_error("BudgetPacer::update_budget: need to call start_pacer() before query budget!");
  }

  if (now_ms < last_update_timestamp_ms_) {
    throw std::runtime_error("BudgetPacer::update_budget: Internal error: now_ms is smaller than last_update_timestamp_ms_");
  }
  
  auto elapsed_ms = now_ms - last_update_timestamp_ms_;
  int increase_bytes = elapsed_ms * pacing_rate_byteps_ / 1000;
  budget_ = min(max_budget_, budget_ + increase_bytes);
  last_update_timestamp_ms_ = now_ms;
}

bool BudgetPacer::ready_to_send(uint32_t now_ms) 
{
  update_budget(now_ms);
  return budget_ >= 0;
}

void BudgetPacer::on_packet_sent(uint32_t timestamp_ms, size_t pkt_size)
{
  budget_ -= pkt_size;
  update_budget(timestamp_ms);
}

TransSender::TransSender(const Address & peer_addr, uint32_t fps,
                         CongestionControlInterface & cc,
                         EncoderInterface & encoder)
  : cc_(cc), encoder_(encoder)
{
  // initialize socket
  socket_.connect(peer_addr);
  socket_.set_timestamps();

  // use fps to initialize the timerfd
  uint32_t BILLION = 1000 * 1000 * 1000;
  const timespec frame_interval {0, static_cast<long>(BILLION / fps)};
  frame_timer_.set_timeout(frame_interval, frame_interval);

  // register actions for poller
  poller_.add_action(Poller::Action(frame_timer_, Direction::In, 
        [this]() -> Result
        {
          // generate new frame and enqueue them into packet queue
          const auto exp = this->frame_timer_.read_expirations();
          if (exp > 1) {
            cerr << "Warning: skip " << exp - 1 << " frames! " << std::endl;
          }

          uint32_t now_ms = timestamp_ms();
          // TODO: change the frame's interval here (need to matain last_encoded_frame_timestamp_ms_)
          auto frame = this->encoder_.encode_next_frame(now_ms);
          auto packets = frame.packets();
          this->data_queue_.insert(this->data_queue_.end(), make_move_iterator(packets.begin()),
                                   make_move_iterator(packets.end()));
          packets.clear();
          return ResultType::Continue;
        }));

  poller_.add_action(Poller::Action(socket_, Direction::Out, 
        [this]() -> Result
        {
          auto now_ms = timestamp_ms();
          // check RTX queue
          if (not this->rtx_queue_.empty()) {
            auto & packet = this->rtx_queue_.front();
            packet.set_send_timestamp_ms(now_ms);
            auto str = packet.to_string();
            this->pacer_.on_packet_sent(now_ms, str.length());
            this->socket_.send(str);
            this->cc_.on_packet_sent(now_ms, packet);
            this->rtx_queue_.pop_front();
            return ResultType::Continue;
          }

          // check data queue
          if (not this->data_queue_.empty()) {
            auto & packet = this->data_queue_.front();
            packet.set_send_timestamp_ms(now_ms);
            auto str = packet.to_string();
            this->pacer_.on_packet_sent(now_ms, str.length());
            this->socket_.send(str);
            this->cc_.on_packet_sent(now_ms, packet);
            // TODO: call RTXMgr::on_packet_send()
            this->data_queue_.pop_front();
            return ResultType::Continue;
          }

          // TODO: shall we do padding here? by something like generate_pad_packet()
          return ResultType::Continue;
        }, 
        [this]() {return this->pacer_.ready_to_send(timestamp_ms()) and this->has_data_to_send();}));
        //[this]() {return this->has_data_to_send() and this->pacer_.ready_to_send(timestamp_ms());}));

  poller_.add_action(Poller::Action(socket_, Direction::In,
        [this]() -> Result
        {
          auto datagram = this->socket_.recv();
          AckPacket ack(datagram.payload);

          auto now_ms = timestamp_ms();
          this->cc_.on_ack_received(now_ms, ack);

          // TODO: call RTXMgr::on_ack_received
          
          return ResultType::Continue;
        }));
}

void TransSender::start() 
{
  pacer_.start_pacer(timestamp_ms());
  while (true) {
    uint32_t ttw = min(pacer_.ms_until_ready(), 1u);
    const auto poll_result = poller_.poll(ttw);
    if (poll_result.result == Poller::Result::Type::Exit) {
      if (poll_result.exit_status) {
        cerr << "Connection error in TransSender::start() (main loop)" << endl;
      }
      throw std::runtime_error("TransSender got a unhandled error in the main loop!");
    }
  }
}

void TransSender::post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps)
{
  pacer_.set_pacing_rate(sending_rate_byteps);
  
  // TODO: calculate RTX rate and subtract it from the target bitrate
  encoder_.set_target_bitrate(target_bitrate_byteps);
}

TransReceiver::TransReceiver(const uint16_t port, DecoderInterface & decoder)
  : decoder_(decoder)
{
  socket_.bind({"0", port});

  poller_.add_action(Poller::Action(socket_, Direction::In, 
      [this]() -> Result
      {
        const auto new_datagram = this->socket_.recv();
        const Packet packet { new_datagram.payload };

        auto now_ms = timestamp_ms();
        auto delay = now_ms - packet.send_timestamp_ms();
        
        this->decoder_.incoming_packet(now_ms, packet);
        AckPacket(packet.connection_id(), packet.frame_no(), packet.fragment_no(), 
            /* delay */ delay, 
            /* curr state */ 0, /* complete state */ {}).sendto(this->socket_, new_datagram.source_address);
        return ResultType::Continue;
      }));
}

void TransReceiver::start()
{
  while (true) {
    const auto poll_result = poller_.poll( -1 );
    if ( poll_result.result == Poller::Result::Type::Exit ) {
      return;
    }
  }
}
