#include <iostream>
#include "congestion_control.hh"

using namespace std;

void CCStatsRecorder::trigger_log(uint32_t timestamp_ms)
{
  if (timestamp_ms- last_log_ms_ > log_period_) {
    last_log_ms_ = timestamp_ms;
    cerr << "[" << timestamp_ms << "] During last " << log_period_ << " ms" << endl
         << "   Sent " << pkt_counter_ << " packets " << size_counter_ << " bytes " << endl
         << "   Received " << ack_counter_ << " acks, avg delay = " << moving_delay_ << " ms" << endl
         << "   Pacing rate = " << sending_rate_ / 125. << "kbps" << endl
         << "   target bitrate = " << target_bitrate_ / 125. << "kbps" << endl;
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
  stats_ = std::make_shared<CCStatsRecorder>();
  add_observer(stats_);
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
  if (not (tao_ms_ >= 0)) return;
  auto i32_tao_ms = max(1, static_cast<int>(tao_ms_)); // at least 1
  int num = d_ms_ / i32_tao_ms - ni_;
  num = max(num, 0); 
  
  //cerr << "curr max pkts = " << num << endl;
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
  stats_->on_packet_sent(timestamp_ms, p);
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
  post_updates();
}

void SalsifyCongestionControl::on_ack_received(uint32_t timestamp_ms, const AckPacket &p)
{
  stats_->on_ack_received(timestamp_ms, p);
  (void)timestamp_ms;
  auto recv_ms = p.arrive_time_ms();
  auto grace_period_ms = query_grace_period(p.frame_no());
  ni_--;
  update_estimation(recv_ms, grace_period_ms);
  post_updates();
}

/* FOR GCC */
void GCCMinus::RateEstimator::on_packet_sent(uint32_t, const Packet & pkt)
{
  SeqNum seq{pkt.frame_no(), pkt.fragment_no()};
  unacked_.insert({seq, pkt});
}

void GCCMinus::RateEstimator::on_ack_received(uint32_t timestamp_ms, const AckPacket & ack)
{
  SeqNum seq{ack.frame_no(), ack.fragment_no()};
  auto it = unacked_.find(seq);
  if (it == unacked_.end()) {
    return;
  }

  uint32_t sz = it->second.to_string().length();
  uint32_t arrive_time_ms = ack.arrive_time_ms();
  acked_bytes_.insert({arrive_time_ms, sz});
  //cerr << "New arrival packet: " << arrive_time_ms << " size: " << sz << endl;

  // update every UPDATE_INTERVAL_MS ms
  if (last_update_ms_ > 0 and timestamp_ms - last_update_ms_ < UPDATE_INTERVAL_MS) {
    return;
  }

  // remove old acks
  uint32_t start_time = arrive_time_ms > WINDOW_MS ? arrive_time_ms - WINDOW_MS : 0;
  for (auto it = acked_bytes_.begin(); it != acked_bytes_.end(); ) {
    if (it->first < start_time) {
      it = acked_bytes_.erase(it);
      //cerr << "remove arrival packet: " << it->first << " size: " << sz << endl;
    }
    else {
      ++it;
    }
  }

  // calculate the incoming rate
  uint32_t total_bytes = 0;
  for (auto & ent : acked_bytes_) {
    total_bytes += ent.second;
  }

  estimate_rate_byteps_ = total_bytes * 1000 / WINDOW_MS;
}

void GCCMinus::DelayEstimator::new_complete_frame(uint32_t frame_no) 
{
  if (curr_completed_frame_ == 0) {
    // only update curr_completed_frame
    curr_completed_frame_ = frame_no;
    return;
  }

  // if not consecutive, remove old frameinfos and skip 
  if (curr_completed_frame_ + 1 != frame_no) {
    for (auto it = frames_.begin(); it != frames_.end(); ) {
      if (it->first < frame_no) {
        it = frames_.erase(it);
      }
      else {
        ++it;
      }
    }
    curr_completed_frame_ = frame_no;
    return;
  }

  // compute and update the values
  auto old_info = frames_.at(curr_completed_frame_);
  auto new_info = frames_.at(frame_no);
  
  result_.d_send = new_info.t_last - old_info.t_last;
  result_.d_recv = new_info.T_last - old_info.T_last;
  result_.d_size = new_info.total_bytes;
  result_.valid = true;

  // erase old and update curr
  frames_.erase(curr_completed_frame_);
  curr_completed_frame_ = frame_no;
}

void GCCMinus::DelayEstimator::on_packet_sent(uint32_t timestamp_ms, const Packet & pkt) 
{
  uint32_t frame_no = pkt.frame_no();
  
  // if it's a new frame, create a new frame info
  if (frames_.count(frame_no) == 0) {
    FrameInfo new_frame_info;
    new_frame_info.send_completed = false;
    new_frame_info.recv_completed = false;
    new_frame_info.total_bytes = 0;
    frames_.emplace(frame_no, new_frame_info);
  }
  
  // update the frame info
  auto & info = frames_.at(frame_no);
  info.total_bytes += pkt.to_string().length();

  if (pkt.fragment_no() == pkt.fragments_in_this_frame() - 1) {
    info.send_completed = true;
    info.t_last = timestamp_ms;
    info.total_pkts = pkt.fragments_in_this_frame();
    cerr << "Frame send complete: " << pkt.frame_no() << " at " << info.t_last << ", total pkts: " << info.total_pkts << endl;
  }
}

void GCCMinus::DelayEstimator::on_ack_received(uint32_t, const AckPacket & ack)
{
  uint32_t frame_no = ack.frame_no();
  if (frames_.count(frame_no) == 0) {
    cerr << "Warning: GCCMinus::DelayEstimator::on_ack_received: acked a packet but no corresponding frameinfo found!" << endl;
    return;
  }
  
  // TODO: think about packet loss scenario! if the packet in the middle of the frame is lost, the group size may be larger
  auto & info = frames_.at(frame_no);
  //cerr << "ACK frame no: " << ack.frame_no() << " frag no " << ack.fragment_no() << endl;
  if (ack.fragment_no() == info.total_pkts - 1) {
    info.recv_completed = true;
    info.T_last = ack.arrive_time_ms();
    //cerr << "Frame recv complete: " << ack.frame_no() << " at " << info.T_last << endl;
    new_complete_frame(frame_no);
  }
}

void GCCMinus::TrendlineFilter::update(const GCCMinus::DelayEstimator::DelayResult & delay_result)
{
  if (not delay_result.valid) return;
  int d_i = delay_result.d_recv - delay_result.d_send;
  if (not initialized_) { 
    m_i_ = 0;
    initialized_ = true;
  }
  m_i_ = ALPHA * d_i + (1-ALPHA) * m_i_;
}

double GCCMinus::TrendlineFilter::get_estimation() const
{
  if (not initialized_) {
    return 0;
  }
  //cerr << "HERE M_I = " << m_i_ << endl;
  return m_i_;
}

void GCCMinus::OveruseDetector::signal_state_change(GCCMinus::OveruseDetector::Signal signal)
{
  switch(state_)
  {
    case HOLD:
      if (signal == NORMAL) state_ = INCREASE;
      else if (signal == OVERUSE) state_ = DECREASE;
      break;
    case DECREASE:
      if (signal != OVERUSE) state_ = HOLD;
      break;
    case INCREASE:
      if (signal == UNDERUSE) state_ = HOLD;
      else if (signal == OVERUSE) state_ = DECREASE;
  }
}

void GCCMinus::OveruseDetector::update(double m_i)
{
  if (m_i < -GAMMA) {
    signal_state_change(UNDERUSE);
  }
  else if (m_i > GAMMA) {
    signal_state_change(OVERUSE);
  }
  else {
    signal_state_change(NORMAL);
  }
}

void GCCMinus::LossCalculator::on_packet_sent(uint32_t, const Packet & p)
{
  auto seq = get_seq(p);
  unacked_.insert(seq);
}

void GCCMinus::LossCalculator::on_ack_received(uint32_t, const AckPacket & ack)
{
  auto seq = get_seq(ack);
  latest_ack_timestamp_ms_ = max(get<0>(seq), latest_ack_timestamp_ms_);
  unacked_.erase(seq);
  acked_.insert(seq);
  //cerr << "LOSS: acked packet " << get<0>(seq) << " " << get<1>(seq) << " " << get<2>(seq) << endl;
}

double GCCMinus::LossCalculator::get_loss_rate()
{
  if (latest_ack_timestamp_ms_ == 0) {
    return 0;
  }

  // remove unacked packet older than one LOSS_INTERVAL_MS
  uint32_t start_time = latest_ack_timestamp_ms_ > LOSS_INTERVAL_MS ? latest_ack_timestamp_ms_ - LOSS_INTERVAL_MS : 0;
  for (auto it = unacked_.begin(); it != unacked_.end();) {
    auto ts = std::get<0>(*it);
    if (ts < start_time) {
      it = unacked_.erase(it);
    }
    else {
      ++it;
    }
  }

  // remove acked packet older than one LOSS_INTERVAL_MS
  for (auto it = acked_.begin(); it != acked_.end();) {
    auto ts = std::get<0>(*it);
    if (ts < start_time) {
      it = acked_.erase(it);
    }
    else {
      ++it;
    }
  }

  // calculate how many packets are unacked in the window
  uint32_t total_lost = 0;
  for (auto & seq : unacked_) {
    if (std::get<0>(seq) < latest_ack_timestamp_ms_) {
      total_lost += 1;
    }
    else {
      break;
    }
  }

  // calculate how many packets are acked in the window
  uint32_t total_acked = acked_.size();

  //cerr << "LOSS: From " << start_time << " to " << latest_ack_timestamp_ms_ << ": " 
  //     << total_lost << " packets are lost and " << total_acked << "packets are acked!" << endl;

  return 1.0f * total_lost / (total_lost + total_acked);
}

void GCCMinus::AimdController::update_estimation(uint32_t now_ms) 
{
  if (last_rate_update_ms_ > 0 and now_ms - last_rate_update_ms_ < TRIGGER_INTERVAL_MS) return;
  last_rate_update_ms_ = now_ms;

  uint32_t as = latest_rate_estimation_byteps_, ar = latest_rate_estimation_byteps_;

  // estimate As
  if (curr_loss_rate_ < LOW_LOSS_THRESH) {
    as = 1.08 * as + 1000;
  }
  else if (curr_loss_rate_ < HIGH_LOSS_THRESH) {
    // do nothing if loss is good
  }
  else {
    as = (1 - 0.5 * curr_loss_rate_) * as;
  }

  // estimate Ar
  switch (curr_state_) {
    case HOLD:
      break;

    case INCREASE:
      ar = 1.08 * ar;
      break;

    case DECREASE:
      ar = 0.85 * incoming_rate_byteps_;
      break;

    default:
      break;
  }

  latest_rate_estimation_byteps_ = min(as, ar);
  cerr << "Ar = " << ar << " as = " << as << endl;
  if (latest_rate_estimation_byteps_ < MIN_RATE_BYTEPS) {
    latest_rate_estimation_byteps_ = MIN_RATE_BYTEPS;
  }
}

uint32_t GCCMinus::AimdController::get_rate_estimation(uint32_t now_ms)
{
  update_estimation(now_ms);
  return latest_rate_estimation_byteps_;
}

void GCCMinus::AimdController::update_loss(uint32_t timestamp_ms, double loss_rate)
{
  curr_loss_rate_ = loss_rate;
  update_estimation(timestamp_ms);
}

void GCCMinus::AimdController::update_state(uint32_t timestamp_ms, RateControlState state)
{
  curr_state_ = state;
  update_estimation(timestamp_ms);
}

void GCCMinus::AimdController::update_incoming_rate(uint32_t timestamp_ms, uint32_t incoming_rate_byteps)
{
  incoming_rate_byteps_ = incoming_rate_byteps;
  update_estimation(timestamp_ms);
}

GCCMinus::GCCMinus()
{
  stats_ = make_shared<CCStatsRecorder>();
  add_observer(stats_);
}

void GCCMinus::post_updates(uint32_t now_ms)
{
  auto estimated_rate = aimd_.get_rate_estimation(now_ms);
  //cerr << "GCC: estimated_rate: " << estimated_rate << endl;
  (void)(now_ms);
  for (auto obs : observers_) {
    //obs->post_updates(2000 * 125, 1900 * 125);
    obs->post_updates(estimated_rate, estimated_rate * 0.9);
  }
}

void GCCMinus::internal_update(uint32_t now_ms)
{
  // TODO: update the heuristics for all components (except LossEst)
  if (last_update_ms_ > 0 and now_ms - last_update_ms_ < UPDATE_INTERVAL_MS) return;

  auto loss_rate = loss_calc_.get_loss_rate();
  //cerr << "HERE: loss rate = " << loss_rate << endl;
  aimd_.update_loss(now_ms, loss_rate);

  auto incoming_rate = rate_est_.get_rate_estimation();
  //cerr << "incoming rate = " << incoming_rate << endl;
  aimd_.update_incoming_rate(now_ms, incoming_rate);

  auto & delay_result = delay_est_.get_result();
  //cerr << "delay_result: " << delay_result.d_recv << " " << delay_result.d_send << " -- " << delay_result.valid << endl;
  if (delay_result.valid) {
    trendline_.update(delay_result);
  }
  double m_i = trendline_.get_estimation();
  //cerr << "m_i: " << m_i << endl;
  
  overuse_.update(m_i);
  aimd_.update_state(now_ms, overuse_.get_state());

  static std::vector<std::string> rate_constrol_str {"HOLD", "INCREASE", "DECREASE"};
  //cerr << "state: " << rate_constrol_str[overuse_.get_state()] << endl;
}

void GCCMinus::on_packet_sent(uint32_t timestamp_ms, const Packet & p) 
{
  stats_->on_packet_sent(timestamp_ms, p);
  loss_calc_.on_packet_sent(timestamp_ms, p);
  delay_est_.on_packet_sent(timestamp_ms, p);
  rate_est_.on_packet_sent(timestamp_ms, p);

  internal_update(timestamp_ms);
  post_updates(timestamp_ms);
}

void GCCMinus::on_ack_received(uint32_t timestamp_ms, const AckPacket & ack)
{
  stats_->on_ack_received(timestamp_ms, ack);
  loss_calc_.on_ack_received(timestamp_ms, ack);
  delay_est_.on_ack_received(timestamp_ms, ack);
  rate_est_.on_ack_received(timestamp_ms, ack);

  internal_update(timestamp_ms);
  post_updates(timestamp_ms);
}
