#include <iostream>
#include "congestion_control.hh"

using namespace std;

void DumbCongestionControl::trigger_log(uint32_t timestamp_ms)
{
  if (timestamp_ms- last_log_ms_ > log_period_) {
    last_log_ms_ = timestamp_ms;
    cerr << "[" << timestamp_ms << "] During last " << log_period_ << " ms" << endl
         << "   Sent " << pkt_counter_ << " packets " << size_counter_ << " bytes " << endl
         << "   Received " << ack_counter_ << " acks, avg delay = " << moving_delay_ << endl;
    pkt_counter_ = 0;
    ack_counter_ = 0;
    size_counter_ = 0;
  }
}

void DumbCongestionControl::on_packet_sent(uint32_t timestamp_ms, const Packet & p)
{
  //cerr << "[" << timestamp_ms  << "] Sent a packet (" 
  //     << p.frame_no() << "," << p.fragment_no() << ")" 
  //     << " size " << p.to_string().length() << endl;
  if (timestamp_ms- last_log_ms_ > log_period_) {
    trigger_log(timestamp_ms);
  }
  
  pkt_counter_ += 1;
  size_counter_ += p.to_string().length();

  for (auto obs : observers_) {
    obs->post_updates(1000*125, 800*125);
  }
}

void DumbCongestionControl::on_ack_received(uint32_t timestamp_ms, const AckPacket & ack)
{
  //cerr << "[" << timestamp_ms << "] received and ack (" 
  //     << ack.frame_no() << "," << ack.fragment_no() << ")" << endl;

  if (timestamp_ms- last_log_ms_ > log_period_) {
    trigger_log(timestamp_ms);
  }

  (void)(ack);
  ack_counter_ += 1;
  moving_delay_ = ack.avg_delay() * 0.2 + moving_delay_ * 0.8;
  for (auto obs : observers_) {
    obs->post_updates(1000*125, 800*125);
  }
}
