#include <iostream>
#include "congestion_control.hh"

using namespace std;

void CCStatsRecorder::trigger_log(uint32_t timestamp_ms)
{
  if (timestamp_ms- last_log_ms_ > log_period_) {
    last_log_ms_ = timestamp_ms;
    cerr << "[" << timestamp_ms << "] During last " << log_period_ << " ms" << endl
         << "   Sent " << pkt_counter_ << " packets " << size_counter_ << " bytes " << endl
         << "   Received " << ack_counter_ << " acks, avg delay = " << moving_delay_ << " ms" << endl;
    pkt_counter_ = 0;
    ack_counter_ = 0;
    size_counter_ = 0;
  }
}

void CCStatsRecorder::on_packet_sent(uint32_t timestamp_ms, const Packet & p)
{
  if (timestamp_ms - last_log_ms_ > log_period_) {
    trigger_log(timestamp_ms);
  }
  pkt_counter_ += 1;
  size_counter_ += p.to_string().length();
}

void CCStatsRecorder::on_ack_received(uint32_t timestamp_ms, const AckPacket & ack)
{
  if (timestamp_ms- last_log_ms_ > log_period_) {
    trigger_log(timestamp_ms);
  }

  (void)(ack);
  ack_counter_ += 1;
  moving_delay_ = ack.avg_delay() * 0.2 + moving_delay_ * 0.8;
}

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

SalsifyCongestionControl::SalsifyCongestionControl(uint32_t D_ms, uint16_t fps)
  : d_ms_(D_ms), fps_(fps)
{
}

void SalsifyCongestionControl::update_grace_period(uint32_t frame_id, uint32_t value_ms)
{
  if (grace_periods_.count(frame_id) == 0) {
    grace_periods_[frame_id] = value_ms;
  }
}

uint32_t SalsifyCongestionControl::query_grace_period(uint32_t frame_id) 
{
  if (grace_periods_.count(frame_id)) {
    auto ret = grace_periods_[frame_id];
    grace_periods_.erase(frame_id);     // each grace_period can only be used once
    return ret;
  }
  return 0;
}

void SalsifyCongestionControl::update_estimation(uint32_t recv_timestamp_ms, uint32_t grace_period_ms)
{
  assert(recv_timestamp_ms >= last_recv_ms_);
  if (tao_ms_ <= 0) {
    tao_ms_ = 0;  // skip the first packet
  }
  else { 
    auto new_value = max(0l, static_cast<int64_t>(recv_timestamp_ms - last_recv_ms_ - grace_period_ms));
    tao_ms_ = ALPHA * new_value + (1 - ALPHA) * tao_ms_;
  }
  last_recv_ms_ = recv_timestamp_ms;
}

void SalsifyCongestionControl::post_updates()
{
  assert(tao_ms_ >= 0);
  auto i32_tao_ms = max(1, static_cast<int>(tao_ms_)); // at least 1
  int num = d_ms_ / i32_tao_ms - ni_;
  num = max(num, 0); 
  
  uint32_t exp_frame_size = num * MTU;
  uint32_t bitrate_byteps = exp_frame_size * fps_;
  uint32_t pacing_byteps = bitrate_byteps * 5;

  //cerr << "SalsifyCongestionControl::post_update: target bitrate bps: " << bitrate_byteps
  //     << " pacing rate bps: " << pacing_byteps << endl;

  for (auto obs : observers_) {
    obs->post_updates(pacing_byteps, bitrate_byteps);
  }
}

void SalsifyCongestionControl::on_packet_sent(uint32_t timestamp_ms, const Packet &p) 
{
  stats_.on_packet_sent(timestamp_ms, p);
  if (last_sent_ms_ <= 0) {
    // initialize the states
    last_sent_ms_ = timestamp_ms;
    curr_frame_id_ = p.frame_no();
  }

  // if it's the current frame, increase ni, update last send ms
  if (p.frame_no() == curr_frame_id_) {
    last_sent_ms_ = timestamp_ms;
    ni_++;
  }
  else {
    // new frame come, update grace period, and then update states
    auto grace_period_ms = timestamp_ms - last_sent_ms_;
    update_grace_period(p.frame_no(), grace_period_ms);

    curr_frame_id_ = p.frame_no();
    last_sent_ms_ = timestamp_ms;
    ni_++;
  }
}

void SalsifyCongestionControl::on_ack_received(uint32_t timestamp_ms, const AckPacket &p)
{
  stats_.on_ack_received(timestamp_ms, p);
  (void)timestamp_ms;
  auto recv_ms = p.arrive_time_ms();
  auto grace_period_ms = query_grace_period(p.frame_no());
  ni_--;
  update_estimation(recv_ms, grace_period_ms);
  post_updates();
}


