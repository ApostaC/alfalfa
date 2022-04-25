#include <thread>
#include <cassert>
#include <iostream>
#include "comp_encoder.hh"
#include "comp_decoder.hh"
#include "congestion_control.hh"
#include "frame_observer.hh"
#include "transmission.hh"

#ifdef NDEBUG
#undef NDEBUG
#endif

using namespace std; 

ostream & out_green(ostream & out)
{
  out << "\033[32m";
  return out;
}

ostream & out_red(ostream & out)
{
  out << "\033[31m";
  return out;
}

ostream & out_yellow(ostream & out)
{
  out << "\033[33m";
  return out;
}

ostream & out_normal(ostream & out)
{
  out << "\033[0m";
  return out;
}

bool test_basic_encoder_decoder()
{
  out_green(cerr) << "============== test: basic encoder decoder ================="; out_normal(cerr) << endl;
  BasicEncoder encoder(500 * 125, 25);
  NonBlockingDecoder decoder;
  auto frame_obs = std::make_shared<FrameArrivalTimeObserver>();
  decoder.add_frame_observer(frame_obs);

  auto frame1 = move(encoder.encode_next_frame(0).get());
  encoder.set_target_bitrate(1000 * 125);
  auto frame2 = move(encoder.encode_next_frame(40).get());

  assert(frame1.frame_no() == 1);
  assert(frame2.frame_no() == 2);

  cerr << "Frame1: " << frame1.packets().size() << " packets" << endl;
  cerr << "Frame2: " << frame2.packets().size() << " packets" << endl;

  for (auto & pkt : frame1.packets()) {
    decoder.incoming_packet(pkt.fragment_no(), pkt);
  }

  for (auto & pkt : frame2.packets()) {
    decoder.incoming_packet(40 + pkt.fragment_no(), pkt);
  }
  
  const auto & arrival_time = frame_obs->arrival_time();
  (void)(arrival_time);
  assert(arrival_time.size() == 2);
  cerr << arrival_time.at(1) << " " << arrival_time.at(2) << endl;
  assert(arrival_time.at(1) == frame1.packets().size() - 1);
  assert(arrival_time.at(2) == frame2.packets().size() - 1 + 40);

  auto packet = frame1.packets()[0];
  uint32_t send_ms = 14214;
  packet.set_send_timestamp_ms(send_ms);
  cerr << "send_timestamp_ms for origin packet is: " << packet.send_timestamp_ms() << endl;
  assert(packet.send_timestamp_ms() == send_ms);

  auto str = packet.to_string();
  Packet new_pkt {str};
  cerr << "send_timestamp_ms for rec packet is: " << new_pkt.send_timestamp_ms() << endl;
  assert(new_pkt.send_timestamp_ms() == send_ms);
  out_green(cerr) << "-------------- PASSED: basic encoder decoder ----------------"; out_normal(cerr) << endl;
  return true;
}

bool test_rtx_mgr()
{
  out_green(cerr) << "============== test: rtx_mgr ================="; out_normal(cerr) << endl;

  BasicEncoder encoder(2500 * 125, 25);
  auto pkts = encoder.encode_next_frame(0).get().packets();
  cerr << "Got " << pkts.size() << " packets!" << endl;
  assert(pkts.size() > 3);

  RTXManager rtx_mgr;

  // send all the packet
  uint32_t ts = 1;
  for(auto & pkt: pkts) {
    pkt.set_send_timestamp_ms(ts);
    rtx_mgr.on_packet_sent(ts, pkt);
    ts += 5;
  }
  
  // receive ack for the first 2 packets
  std::deque<Packet> rtx_queue {};
  uint32_t OWD = 30;
  auto receive_packet = [&](int i) {
    auto & pkt = pkts[i];
    AckPacket ack(pkt.connection_id(), pkt.frame_no(), pkt.fragment_no(),
          0, 0, {});
    ack.set_send_time(pkt.send_timestamp_ms());
    ack.set_arrive_time(pkt.send_timestamp_ms() + OWD);
    rtx_mgr.on_ack_received(pkt.send_timestamp_ms() + 2 * OWD, ack, rtx_queue);
  };

  for(int i=0;i<2;i++) {
    receive_packet(i);
    assert(rtx_queue.size() == 0); // no retransmission here!
  }

  // receive the ack for the last packet
  receive_packet(pkts.size() - 1);
  cerr << "Got " << rtx_queue.size() << " rtx packets!" << endl;
  assert(rtx_queue.size() > 0);
  out_green(cerr) << "-------------- PASSED: rtx_mgr ----------------"; out_normal(cerr) << endl;
  return true;
}

bool test_blocking_decoder()
{
  out_green(cerr) << "============== test: blocking decoder ================="; out_normal(cerr) << endl;
  BasicEncoder encoder(500 * 125, 25);
  encoder.set_gop(3);
  std::vector<FragmentedFrame> frames;
  for (int i=0;i<4;i++) {
    frames.emplace_back(move(encoder.encode_next_frame(0).get()));
    cerr << "Frame " << frames.back().frame_no() << " is_key_frame = " << frames.back().is_key_frame() 
         << " number of packets: " << frames.back().fragments_in_this_frame() << endl;
    if (i % 3 == 0) {
      assert(frames.back().is_key_frame());
    }
  }

  BlockingDecoder decoder;
  auto obs = std::make_shared<FrameArrivalTimeObserver>();
  decoder.add_frame_observer(obs);

  // ts = 0, frame 1 is not complete
  decoder.incoming_packet(0, frames[0].packets()[0]);

  // ts = 1, frame 2 is complete
  for (auto pkt : frames[1].packets()) {
    decoder.incoming_packet(1, pkt);
  }

  // ts = 2, frame 1 is complete
  for (auto pkt : frames[0].packets()) {
    decoder.incoming_packet(2, pkt);
  }

  // ts = 3, frame 3 is incomplete
  decoder.incoming_packet(3, frames[2].packets()[0]);

  // ts = 4, frame 4 is incomplete
  for (auto pkt : frames[3].packets()) {
    decoder.incoming_packet(4, pkt);
  }


  auto final_result = obs->arrival_time();
  decltype(final_result) expected_result{{1,2}, {2,2}, {4,4}};
  assert(final_result.size() == expected_result.size());
  for(auto ent: final_result) {
    assert(expected_result.count(ent.first) > 0);
    auto exp_res = expected_result[ent.first];
    cerr << "Frame " << ent.first << " arrives at " << ent.second << endl;
    cerr << "expected_result " << ent.first << " arrives at " << exp_res << endl;
    assert(exp_res == ent.second);
  }
  out_green(cerr) << "-------------- PASSED: blocking decoder ----------------"; out_normal(cerr) << endl;
  return true;
}

bool test_trans()
{
  out_green(cerr) << "============== test: trans ================="; out_normal(cerr) << endl;

  uint32_t fps = 25;
  BasicEncoder encoder(500 * 125, 25);
  NonBlockingDecoder decoder;
  //DumbCongestionControl cc;
  SalsifyCongestionControl cc(100, fps);
  NoRTX rtx;
  auto frame_obs = std::make_shared<FrameArrivalTimeObserver>();
  decoder.add_frame_observer(frame_obs);

  uint16_t port = 54123;
  std::string ip = "127.0.0.1";
  auto sender = std::make_shared<TransSender>(Address(ip, port), fps, std::ref(cc), std::ref(encoder), std::ref(rtx));
  auto receiver = std::make_shared<TransReceiver>(port, decoder);

  cc.add_observer(sender);

  //sender->start();
  std::thread receiver_thread([&](){receiver->start();});
  std::thread sender_thread([&](){sender->start();});

  receiver_thread.join();
  sender_thread.join();

  out_green(cerr) << "-------------- PASSED: trans ----------------"; out_normal(cerr) << endl;
  return true;
}

int main(int argc, char *argv[])
{
  (void)(argc);
  (void)(argv);
  out_green(cerr) << "Starting test!";
  out_normal(cerr) << endl;

  test_basic_encoder_decoder();
  test_blocking_decoder();
  test_rtx_mgr();
  test_trans();
}
