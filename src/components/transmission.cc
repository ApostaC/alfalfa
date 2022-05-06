#include <iostream>
#include <thread>
#include "timestamp.hh"
#include "transmission.hh"

//TODO: Pacer is in milliseconds, so the MAX rate will be limited to MTU * 1000 byte per second!
using namespace std;
using namespace PollerShortNames;

RTXManager::RTXManager() 
{
}

void RTXManager::add_unacked(const Packet & pkt)
{
  SeqNum seq{pkt.frame_no(), pkt.fragment_no()};
  if (unacked_.count(seq)) {
    cerr << "Warning: RTXManager::add_unacked: adding an unacked packet with the same sequence number!" << endl;
    return;
  }
  
  UnackedInfo info{pkt, 0, 0};
  unacked_.emplace(seq, info);
}

void RTXManager::add_rtt_sample(uint32_t rtt_ms)
{
  if (not ewma_rtt_ms_.initialized()) {
    ewma_rtt_ms_ = rtt_ms;
  }
  else {
    ewma_rtt_ms_ = ALPHA * rtt_ms + (1 - ALPHA) * ewma_rtt_ms_.get();
  }
}


void RTXManager::on_packet_sent(uint32_t, const Packet & pkt)
{
  assert(pkt.send_timestamp_ms() != 0);
  add_unacked(pkt);
}

void RTXManager::on_ack_received(uint32_t timestamp_ms, const AckPacket & ack, deque<Packet> & target_buf)
{
  // update rtt
  int rtt = timestamp_ms - ack.send_time_ms();
  assert(rtt >= 0);
  add_rtt_sample(rtt);

  // find the unacked packet, return if nothing found
  SeqNum seq{ack.frame_no(), ack.fragment_no()};
  auto acked_it = unacked_.find(seq);
  if (acked_it == unacked_.end()) {
    return;
  }
  
  // retransmit all the unacked packet before the acked ones
  for (auto rit = make_reverse_iterator(acked_it); 
       rit != unacked_.rend(); ++rit) {
    auto & info = rit->second;
    
    // if not retransmitted or retransmission is 1 RTT ago and not in the waiting queue
    // TODO: problem here: last sent ms is the enqueue time, not the sending time, 
    // so a packet may be enqueue twice or more before being retransmitted
    if (info.num_rtx == 0 or
        timestamp_ms - info.last_sent_ms > ewma_rtt_ms_.get()) {
        //(timestamp_ms - info.last_sent_ms > ewma_rtt_ms_.get() and
        // info.is_waiting == false)) {
      info.num_rtx++;
      info.last_sent_ms = timestamp_ms;
      info.is_waiting = true;
      target_buf.push_front(info.packet);
    }
  }

  // erase the acked packet information
  unacked_.erase(acked_it);
}

void RTXManager::on_rtx_sent(uint32_t timestamp_ms, const Packet & pkt) 
{
  SeqNum seq{pkt.frame_no(), pkt.fragment_no()};
  if (unacked_.count(seq) != 0){  // skip if it's alread acked!
    auto info = unacked_.at(seq);
    //info.last_sent_ms = timestamp_ms;
    info.is_waiting = false;
  }
  sent_rtx_size_.emplace(timestamp_ms, pkt.to_string().length());
}

uint32_t RTXManager::get_rtx_bitrate_byteps(uint32_t timestamp_ms)
{
  auto start_it = sent_rtx_size_.lower_bound(timestamp_ms - RTX_RATE_WINDOW_MS);
  auto end_it = sent_rtx_size_.upper_bound(timestamp_ms);
  uint32_t tot_size_bytes = 0;
  for(auto it = start_it; it != end_it; ++it) {
    tot_size_bytes += it->second;
  }
  return tot_size_bytes * 1000 / RTX_RATE_WINDOW_MS;
}

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

void BudgetPacer::set_pacing_rate(uint32_t rate_byteps)
{ 
  pacing_rate_byteps_ = rate_byteps; 
  max_budget_ = (int)pacing_rate_byteps_ / 1000;
  if (max_budget_ < DEFAULT_MAX_BUDGET) {
    max_budget_ = DEFAULT_MAX_BUDGET;
  }
}

void BudgetPacer::on_packet_sent(uint32_t timestamp_ms, size_t pkt_size)
{
  budget_ -= pkt_size;
  update_budget(timestamp_ms);
}

TransSender::TransSender(const Address & peer_addr, uint32_t fps,
                         CongestionControlInterface & cc,
                         EncoderInterface & encoder, 
                         RTXInterface & rtx_mgr)
  : cc_(cc), encoder_(encoder), rtx_mgr_(rtx_mgr)
{
  // initialize socket
  socket_.connect(peer_addr);
  socket_.set_timestamps();

  // use fps to initialize the timerfd
  uint32_t BILLION = 1000 * 1000 * 1000;
  const timespec frame_interval {0, static_cast<long>(BILLION / fps)};
  frame_timer_.set_timeout(frame_interval, frame_interval);

  // register actions for poller
  // activating the encoder
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
          //auto old_queue_length = data_queue_.size();
          if (frame.initialized())
          {
            auto packets = frame.get().packets();
            this->data_queue_.insert(this->data_queue_.end(), make_move_iterator(packets.begin()),
                                     make_move_iterator(packets.end()));
            packets.clear();
          }

          this->pacer_.set_pacing_rate(this->cached_pacing_rate_);
          cerr << "[" << now_ms << "] Queue length is " << data_queue_.size() 
               << " cached pacing rate is " << this->cached_pacing_rate_ << endl;
               //<< "   old queue len = " << old_queue_length << endl
               //<< "   rtx queue len = " << rtx_queue_.size() << endl;
          return ResultType::Continue;
        }));

  // sending out the packets
  poller_.add_action(Poller::Action(socket_, Direction::Out, 
        [this]() -> Result
        {
          auto now_ms = timestamp_ms();
          while(this->pacer_.ready_to_send(now_ms) and this->has_data_to_send()) {
            this->send_one_packet(now_ms);
            now_ms = timestamp_ms();
          }

          return ResultType::Continue;
        }, 
        [this]() {return this->pacer_.ready_to_send(timestamp_ms()) and this->has_data_to_send();}));
        //[this]() {return this->has_data_to_send() and this->pacer_.ready_to_send(timestamp_ms());}));

  // receive the acks
  poller_.add_action(Poller::Action(socket_, Direction::In,
        [this]() -> Result
        {
          auto datagram = this->socket_.recv();
          AckPacket ack(datagram.payload);

          auto now_ms = timestamp_ms();
          this->cc_.on_ack_received(now_ms, ack);

          this->rtx_mgr_.on_ack_received(now_ms, ack, this->rtx_queue_);
          return ResultType::Continue;
        }));
}

void TransSender::send_one_packet(uint32_t now_ms)
{
  // check RTX queue
  if (not this->rtx_queue_.empty()) {
    auto & packet = this->rtx_queue_.front();
    packet.set_send_timestamp_ms(now_ms);
    auto str = packet.to_string();
    this->pacer_.on_packet_sent(now_ms, str.length());
    this->socket_.send(str);
    this->cc_.on_packet_sent(now_ms, packet);
    this->rtx_mgr_.on_rtx_sent(now_ms, packet);
    cerr << "[" << now_ms << "] retrainsmitted packet " << packet.frame_no() << "," << packet.fragment_no() << endl;
    this->rtx_queue_.pop_front();
    return;
  }

  // check data queue
  if (not this->data_queue_.empty()) {
    auto & packet = this->data_queue_.front();
    packet.set_send_timestamp_ms(now_ms);
    auto str = packet.to_string();
    this->pacer_.on_packet_sent(now_ms, str.length());
    this->socket_.send(str);
    this->cc_.on_packet_sent(now_ms, packet);
    this->rtx_mgr_.on_packet_sent(now_ms, packet);
    this->data_queue_.pop_front();
    return;
  }
  // TODO: shall we do padding here? by something like generate_pad_packet()
}

void TransSender::send_stop_message() 
{
  Packet pkt;
  pkt.set_fragments_in_this_frame(1);
  pkt.set_stop();
  try {
  this->socket_.send(pkt.to_string());
  } catch (exception & e) {
    cerr << "skip this one because error occured: " << e.what() << endl;
  }
}

void TransSender::start(uint32_t time_limit_ms) 
{
  uint32_t start_ms = timestamp_ms();
  pacer_.start_pacer(start_ms);
  while (true) {
    uint32_t ttw = min(pacer_.ms_until_nextcheck(), 1u);
    const auto poll_result = poller_.poll(ttw);
    if (poll_result.result == Poller::Result::Type::Exit) {
      if (poll_result.exit_status) {
        cerr << "Connection error in TransSender::start() (main loop)" << endl;
      }
      throw std::runtime_error("TransSender got a unhandled error in the main loop!");
    }

    if(time_limit_ms > 0 and timestamp_ms() > start_ms + time_limit_ms) {
      cerr << "Time limit reached! stop..." << endl;
      break;
    }
  }
  
  /* send 10 stop message */
  cerr << "Sending stop message ..." << endl;
  for(int i=0;i<50;i++) {
    using namespace std::chrono_literals;
    send_stop_message();
    this_thread::sleep_for(5ms);
  }
}

void TransSender::post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps, double loss_rate)
{
  cached_pacing_rate_ = sending_rate_byteps;
  //pacer_.set_pacing_rate(sending_rate_byteps);
  // NOTE: if we use pacer.set_pacing_rate here, it will cause the real pacing rate 
  //       of a frame does not match the expected pacing rate when encoding the frame
  // TODO: use loss rate here
  
  auto now_ms = timestamp_ms();
  auto rtx_rate = rtx_mgr_.get_rtx_bitrate_byteps(now_ms);
  int tgt_rate = target_bitrate_byteps - rtx_rate;
  tgt_rate = max(0, tgt_rate);
  encoder_.set_target_bitrate(tgt_rate);
  encoder_.set_loss_rate(loss_rate);
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

        if (packet.control_signal() == Packet::STOP) {
          this->stop_received_ = true;
          return ResultType::Continue;
        }

        auto now_ms = timestamp_ms();
        auto delay = now_ms - packet.send_timestamp_ms();
        
        this->decoder_.incoming_packet(now_ms, packet);
        AckPacket ack(packet.connection_id(), packet.frame_no(), packet.fragment_no(), 
            /* delay */ delay, 
            /* curr state */ 0, /* complete state */ {}); 
        ack.set_arrive_time(now_ms);
        ack.set_send_time(packet.send_timestamp_ms());
        ack.set_origin_pkt_size(new_datagram.payload.size());
        ack.sendto(this->socket_, new_datagram.source_address);

        return ResultType::Continue;
      }));
}

void TransReceiver::start(uint32_t time_limit_ms)
{
  uint32_t start_ms = timestamp_ms();
  uint32_t timeout_ms = 100;
  while (true) {
    const auto poll_result = poller_.poll( timeout_ms );
    if ( poll_result.result == Poller::Result::Type::Exit ) {
      return;
    }

    if (time_limit_ms != (~0u) and timestamp_ms() > start_ms + time_limit_ms) {
      cerr << "Time limit reached! stop..." << endl;
      break;
    }

    if (stop_received_) {
      cerr << "Stop signal received!" << endl;
      break;
    }
  }
}
