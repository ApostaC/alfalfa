#include <iostream>
#include "congestion_control.hh"

using namespace std;

double BBRMinus::PACING_GAIN_LIST[BBRMinus::PACING_GAIN_LIST_LEN] = {1.33, 0.75, 1, 1, 1, 1};

void CCStatsRecorder::trigger_log(uint32_t timestamp_ms)
{
  if (timestamp_ms- last_log_ms_ > log_period_) {
    last_log_ms_ = timestamp_ms;
    cerr << "[" << timestamp_ms << "] During last " << log_period_ << " ms" << endl
         << "   Sent " << pkt_counter_ << " packets " << size_counter_ << " bytes " << endl
         << "   Received " << ack_counter_ << " acks, avg delay = " << moving_delay_ << " ms" << endl
         << "   est. loss = " << loss_rate_ << endl
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
    obs->post_updates(1000*125, 800*125, 0);
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
    obs->post_updates(1000*125, 800*125, 0);
  }
}

/* CLASS OracleCongestionControl */

void OracleCongestionControl::post_updates()
{
  auto bw = get_bw();
  auto loss = get_loss();
  for (auto obs : observers_) {
    obs->post_updates(bw, bw * 0.9, loss);
  }
}

void OracleCongestionControl::update_real_stats(uint32_t timestamp_ms)
{
  const static uint32_t UPDATE_INTERVAL_MS = 100;
  if (timestamp_ms - last_real_stats_update_ms_ <= UPDATE_INTERVAL_MS) {
    return;
  }

  if (not real_stats_obs_) {
    return;
  }

  auto bw = get_bw();
  auto loss = get_loss();
  real_stats_obs_->post_updates(bw, bw * 0.9, loss);
  last_real_stats_update_ms_ = timestamp_ms;
}

void OracleCongestionControl::on_packet_sent(uint32_t ts, const Packet & p) 
{
  stats_->on_packet_sent(ts, p);
  loss_calc_.on_packet_sent(ts, p);
  update_real_stats(ts);
}

void OracleCongestionControl::on_ack_received(uint32_t ts, const AckPacket & ack) 
{
  stats_->on_ack_received(ts, ack);
  loss_calc_.on_ack_received(ts, ack);
  update_real_stats(ts);
}

void OracleCongestionControl::set_bw(uint32_t bw_byteps) 
{
  {
    unique_lock<mutex> lock(mut_);
    real_bw_byteps_ = bw_byteps;
  }
  post_updates();
}

void OracleCongestionControl::set_loss(double loss_rate)
{
  {
    unique_lock<mutex> lock(mut_);
    real_loss_rate_ = loss_rate;
  }
  post_updates();
}

uint32_t OracleCongestionControl::get_bw()
{
  unique_lock<mutex> lock(mut_);
  return real_bw_byteps_;
}

double OracleCongestionControl::get_loss()
{
  unique_lock<mutex> lock(mut_);
  return real_loss_rate_;
}

/* CLASS SalsifyCongestionControl*/

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

void SalsifyCongestionControl::update_estimation(uint32_t recv_timestamp_ms, uint32_t grace_period_ms, uint32_t pkt_size)
{
  assert(recv_timestamp_ms >= last_recv_ms_);
  if (tao_ms_ < 0) {
    tao_ms_ = 0;  // skip the first packet
  }
  else { 
    (void)pkt_size;
    auto new_value = max(0, static_cast<int>(recv_timestamp_ms - last_recv_ms_ - grace_period_ms));
    //auto new_value = static_cast<int>(recv_timestamp_ms - last_recv_ms_);
    //new_value = new_value * MTU / pkt_size;
    //new_value = max(0, static_cast<int>(new_value - grace_period_ms));
    if (tao_ms_ == 0) { // the first packet
      tao_ms_ = new_value;
    }
    else {
      tao_ms_ = ALPHA * new_value + (1 - ALPHA) * tao_ms_;
    }
  }
  last_recv_ms_ = recv_timestamp_ms;
}

void SalsifyCongestionControl::post_updates()
{
  if (not (tao_ms_ > 0)) return;
  auto i32_tao_ms = max(1, static_cast<int>(tao_ms_)); // at least 1
  int num = d_ms_ / i32_tao_ms - inflight_.size();
  num = max(num, 0); 
  
  uint32_t exp_frame_size = num * MTU;
  uint32_t bitrate_byteps = exp_frame_size * fps_;
  if (bitrate_byteps < MIN_RATE_BYTEPS) {
    bitrate_byteps = MIN_RATE_BYTEPS;
  }
  uint32_t pacing_byteps = bitrate_byteps * 5;

  //cerr << "SalsifyCongestionControl::post_update: target bitrate bps: " << bitrate_byteps
  //     << " pacing rate bps: " << pacing_byteps 
  //     << " available slots: " << d_ms_ / i32_tao_ms << endl;

  auto loss_rate = loss_calc_.get_loss_rate();
  for (auto obs : observers_) {
    obs->post_updates(pacing_byteps, bitrate_byteps, loss_rate);
  }
}

void SalsifyCongestionControl::remove_expired_packets(uint32_t now_ms)
{
  /* need to wait for RTT estimation */
  if (ewma_rtt_ms_ == 0) return; 

  uint32_t earliest_sent_timestamp = now_ms - 3 * ewma_rtt_ms_;
  for(auto it = inflight_.begin(); it != inflight_.end(); ) {
    if (get<0>(*it) < earliest_sent_timestamp) {
      it = inflight_.erase(it);
    }
    else {
      break;
    }
  }
}

void SalsifyCongestionControl::on_packet_sent(uint32_t timestamp_ms, const Packet &p) 
{
  stats_->on_packet_sent(timestamp_ms, p);
  loss_calc_.on_packet_sent(timestamp_ms, p);

  SeqNum seq = get_seq(p);
  inflight_.insert(seq);

  if (last_sent_ms_ <= 0) {
    // initialize the states
    last_sent_ms_ = timestamp_ms;
    curr_frame_id_ = p.frame_no();
  }

  // if it's the current frame, increase ni, update last send ms
  if (p.frame_no() == curr_frame_id_) {
    last_sent_ms_ = timestamp_ms;
  }
  else if (p.frame_no() > curr_frame_id_) {
    // new frame come, update grace period, and then update states
    auto grace_period_ms = timestamp_ms - last_sent_ms_;
    update_grace_period(p.frame_no(), grace_period_ms);

    curr_frame_id_ = p.frame_no();
    last_sent_ms_ = timestamp_ms;
  }
  else {
    last_sent_ms_ = timestamp_ms; 
  }

  //cerr << "[" << timestamp_ms << "] SEND packet " << p.frame_no() 
  //     << " (in flight " << inflight_.size() << ")" << endl;
  remove_expired_packets(timestamp_ms);
  post_updates();
}

void SalsifyCongestionControl::on_ack_received(uint32_t timestamp_ms, const AckPacket &p)
{
  stats_->on_ack_received(timestamp_ms, p);
  loss_calc_.on_ack_received(timestamp_ms, p);

  /* update inflight packets */
  auto seq = get_seq(p);
  inflight_.erase(seq);

  /* update rtt */
  auto rtt = timestamp_ms - p.send_time_ms();
  if (ewma_rtt_ms_ == 0) {
    ewma_rtt_ms_ = rtt;
  }
  else {
    ewma_rtt_ms_ = ALPHA * rtt + (1-ALPHA) * ewma_rtt_ms_;
  }

  /* update grace period ms */
  auto recv_ms = p.arrive_time_ms();
  auto grace_period_ms = query_grace_period(p.frame_no());
  update_estimation(recv_ms, grace_period_ms, p.origin_pkt_size());
  //cerr << "[" << timestamp_ms << "] RECV packet " << p.frame_no() 
  //     << " (arrive at: " << recv_ms << ")"
  //     << " (in flight " << inflight_.size() << ")" << endl;
  remove_expired_packets(timestamp_ms);
  post_updates();
}

/* FOR GCC */
void RateEstimator::on_packet_sent(uint32_t, const Packet & pkt)
{
  SeqNum seq{pkt.frame_no(), pkt.fragment_no()};
  unacked_.insert({seq, pkt});
}

void RateEstimator::on_ack_received(uint32_t timestamp_ms, const AckPacket & ack)
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

void LossCalculator::on_packet_sent(uint32_t, const Packet & p)
{
  auto seq = get_seq(p);
  unacked_.insert(seq);
}

void LossCalculator::on_ack_received(uint32_t, const AckPacket & ack)
{
  auto seq = get_seq(ack);
  latest_ack_timestamp_ms_ = max(get<0>(seq), latest_ack_timestamp_ms_);
  unacked_.erase(seq);
  acked_.insert(seq);
  //cerr << "LOSS: acked packet " << get<0>(seq) << " " << get<1>(seq) << " " << get<2>(seq) << endl;
}

double LossCalculator::get_loss_rate()
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
  auto loss_rate = loss_calc_.get_loss_rate();
  //cerr << "GCC: estimated_rate: " << estimated_rate << endl;
  (void)(now_ms);
  for (auto obs : observers_) {
    //obs->post_updates(2000 * 125, 1900 * 125);
    obs->post_updates(estimated_rate, estimated_rate * 0.9, loss_rate);
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

/* FOR BBR */
BBRMinus::BBRMinus()
{
  /* init the state machine */
  machine_.state = START;
  machine_.next_update_ms = 0;

  /* stats */
  stats_ = make_shared<CCStatsRecorder>();
  add_observer(stats_);
}

void BBRMinus::post_updates()
{
  uint32_t rate = btlbw_.get() * pacing_gain_;
  //get_logger(0) << "Post update: btlbw = " << btlbw_.get() << ", pacing gain = " << pacing_gain_ << ", rate = " << rate << endl;
  auto loss = loss_calc_.get_loss_rate();
  for(auto obs : observers_) {
    obs->post_updates(rate, rate * 0.9, loss);
  }
}

std::ostream & BBRMinus::get_logger(uint32_t timestamp_ms)
{ 
  cerr << "[BBR: " << timestamp_ms << "]: "; 
  return cerr; 
}

void BBRMinus::update_machine(uint32_t timestamp_ms)
{
  /* if it's just initialized */
  if (machine_.next_update_ms == 0) {
    machine_.state = START;
    machine_.next_update_ms = timestamp_ms;
    enter_startup(timestamp_ms);
  }

  if (machine_.next_update_ms > timestamp_ms) {
    return;
  }

  switch (machine_.state) {
    case START:
      execute_startup(timestamp_ms);
      break;
    
    case DRAIN:
      execute_drain(timestamp_ms);
      break;

    case PROBE_BW:
      execute_probebw(timestamp_ms);
      break;

    case PROBE_RTT:
      execute_probertt(timestamp_ms);
      break;

    default:
      throw runtime_error("BBRMinus: unknown machine state");
  };
}

void BBRMinus::enter_startup(uint32_t timestamp_ms)
{
  /**
   * start-up state:
   *   pacing gain = 1.5
   *   normal update: min(rtprop_, 100ms);
   *
   *   if btlbw_ is not updated for 400 ms, go to DRAIN
   *   if rtprop_ is not updated for 10s, goto probertt
   */
  pacing_gain_ = PACING_GAIN_STARTUP;

  /* normal processing */
  machine_.state = START; 
  machine_.next_update_ms = min(rtprop_.get(), 100u) + timestamp_ms;

  get_logger(timestamp_ms) << "enter startup! next update is " << machine_.next_update_ms << endl;
}

void BBRMinus::execute_startup(uint32_t timestamp_ms)
{
  /* if btlbw is not chaing, enter DRAIN */
  if (static_cast<int32_t>(timestamp_ms - btlbw_.last_update_ms()) > 
      static_cast<int32_t>(STARTUP_TO_DRAIN_MS) and btlbw_.initialized()) {
    exit_startup(timestamp_ms);
    enter_drain(timestamp_ms);
    return;
  }

  /* if rtprop is not changing, enter PROBE_RTT */
  if (static_cast<int32_t>(timestamp_ms - rtprop_.last_update_ms()) > 
      static_cast<int32_t>(PROBERTT_ENTER_MS) and rtprop_.initialized()) {
    exit_startup(timestamp_ms);
    enter_probertt(timestamp_ms);
    return;
  }
  
  /* normal processing */
  pacing_gain_ = PACING_GAIN_STARTUP;
  machine_.next_update_ms = min(rtprop_.get(), 100u) + timestamp_ms;
  get_logger(timestamp_ms) << "execute startup! next update is " << machine_.next_update_ms << endl;
}

void BBRMinus::exit_startup(uint32_t)
{
  /* nothing to do */
}

void BBRMinus::enter_drain(uint32_t timestamp_ms)
{
  /**
   * drain state:
   *  pacing gain = 0.3
   *  normal update: min(rtprop_, 100ms)
   *
   *  if bytes in flight is smaller than bdp, enter PROBE_BW
   */
  pacing_gain_ = PACING_GAIN_DRAIN;
  machine_.next_update_ms = min(rtprop_.get(), 100u) + timestamp_ms;
  machine_.state = DRAIN;

  drain_round_count_ = 0;
  get_logger(timestamp_ms) << "enter drain!" << endl;
}

void BBRMinus::execute_drain(uint32_t timestamp_ms)
{
  uint32_t in_flight = 0;
  for (auto & ent : unacked_) {
    in_flight += ent.second.size_in_bytes;
  }
  
  auto bdp = btlbw_.get() * rtprop_.get() / 1000;
  if (in_flight <= bdp or drain_round_count_ >= 8) {
    exit_drain(timestamp_ms);
    enter_probebw(timestamp_ms);
    return;
  }

  machine_.next_update_ms = min(rtprop_.get(), 100u) + timestamp_ms;
  drain_round_count_ ++;
  get_logger(timestamp_ms) << "execute drain! bdp = " << bdp << ", inflight = " << in_flight << endl;
}

void BBRMinus::exit_drain(uint32_t)
{
  drain_round_count_ = 0;
}

void BBRMinus::enter_probebw(uint32_t timestamp_ms)
{
  /**
   * probe_bw state:
   *   pacing gain = gain_list[round % N]
   *   normal update: min(rtprop_, 150ms)
   *
   *   if rtprop_ is not updated for 10s, goto probertt
   */

  probebw_round_count_ = 0;
  pacing_gain_ = PACING_GAIN_LIST[0];

  machine_.next_update_ms = min(rtprop_.get(), 150u) + timestamp_ms;
  machine_.state = PROBE_BW;
  get_logger(timestamp_ms) << "enter probe_bw!" << endl;
}

void BBRMinus::execute_probebw(uint32_t timestamp_ms) 
{
  /* if rtprop is not changing, enter PROBE_RTT */
  if (static_cast<int32_t>(timestamp_ms - rtprop_.last_update_ms()) > 
      static_cast<int32_t>(PROBERTT_ENTER_MS) and rtprop_.initialized()) {
    exit_startup(timestamp_ms);
    enter_probertt(timestamp_ms);
    return;
  }

  probebw_round_count_ ++;
  pacing_gain_ = PACING_GAIN_LIST[probebw_round_count_ % PACING_GAIN_LIST_LEN];
  machine_.next_update_ms = min(rtprop_.get(), 150u) + timestamp_ms;
}

void BBRMinus::exit_probebw(uint32_t)
{
  probebw_round_count_ = 0;
}

void BBRMinus::enter_probertt(uint32_t timestamp_ms)
{
  /**
   * probe_rtt state:
   *   pacing gain = 0.3
   *   normal update: max(rtt, 150) // after this update, it will enter another state
   *
   *   if bytes in flight is not full, goto start
   *   if bytes in flight is full, goto probebw
   */
  pacing_gain_ = PACING_GAIN_PROBERTT;
  probertt_start_ = timestamp_ms;

  machine_.next_update_ms = max(rtprop_.get(), 150u) + timestamp_ms;
  machine_.state = PROBE_RTT;
  get_logger(timestamp_ms) << "enter probe_rtt!" << endl;
}

void BBRMinus::execute_probertt(uint32_t timestamp_ms)
{
  uint32_t in_flight = 0;
  auto bdp = btlbw_.get() * rtprop_.get() / 1000;
  for (auto & ent : unacked_) {
    in_flight += ent.second.size_in_bytes;
  }
  
  if (in_flight <= bdp) {
    exit_probertt(timestamp_ms);
    enter_probebw(timestamp_ms);
    return;
  }
  else {
    exit_probertt(timestamp_ms);
    enter_probebw(timestamp_ms);
    return;
  }
}

void BBRMinus::exit_probertt(uint32_t)
{
  /* do nothing here */
}

void BBRMinus::clear_expired_packets(uint32_t timestamp_ms) 
{
  if (last_clear_ms_ == 0) {
    last_clear_ms_ = timestamp_ms;
    return;
  }

  /* execute clear every 1 sec */
  if (not timestamp_ms - last_clear_ms_ > 1000) {
    return;
  }

  /* clear the all the packet information before last clear */
  for (auto it = unacked_.begin(); it != unacked_.end();) {
    if (it->second.timestamp_ms < last_clear_ms_) {
      it = unacked_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = in_retrans_.begin(); it != in_retrans_.end();) {
    if (it->second.timestamp_ms < last_clear_ms_) {
      it = in_retrans_.erase(it);
    } else {
      ++it;
    }
  }

  last_clear_ms_ = timestamp_ms;
}

void BBRMinus::on_packet_sent(uint32_t timestamp_ms, const Packet &p)
{
  stats_->on_packet_sent(timestamp_ms, p);

  /* update unacked and in_retrans */
  SeqNum seq{p.frame_no(), p.fragment_no()};
  PktInfo pi;
  pi.size_in_bytes = p.to_string().length();
  pi.timestamp_ms = timestamp_ms;
  unacked_.insert({seq, pi});
  if (p.is_retrans()) {
    in_retrans_.insert({seq, pi});
  }
  
  /* update loss and rate estimator */
  rate_est_.on_packet_sent(timestamp_ms, p);
  loss_calc_.on_packet_sent(timestamp_ms, p);

  update_machine(timestamp_ms);
  clear_expired_packets(timestamp_ms);
  post_updates();
}

void BBRMinus::on_ack_received(uint32_t timestamp_ms, const AckPacket &p)
{
  stats_->on_ack_received(timestamp_ms, p);
  rate_est_.on_ack_received(timestamp_ms, p);
  loss_calc_.on_ack_received(timestamp_ms, p);

  /* update rtt and unacked */
  SeqNum seq{p.frame_no(), p.fragment_no()};
  auto ack_it = unacked_.find(seq);

  if (ack_it != unacked_.end()) {
    if (in_retrans_.count(seq) == 0) {
      /* only update RTT for non-retrans packets */
      auto rtt = timestamp_ms - p.send_time_ms();
      rtprop_.update(rtt, timestamp_ms);
      //cerr << "HERE RTT = " << rtt << endl;
    }

    /* remove from unacked */
    unacked_.erase(ack_it);
  }

  /* update btlbw every 50 ms */
  if (last_bw_est_ == 0) {
    last_bw_est_ = timestamp_ms;
  }
  if (timestamp_ms - last_bw_est_ > 50) {
    btlbw_.set_window(10 * rtprop_.get());
    btlbw_.update(rate_est_.get_rate_estimation(), timestamp_ms);
    get_logger(timestamp_ms) << "Update btlbw with value: " << rate_est_.get_rate_estimation() 
                             << ", now btlbw is " << btlbw_.get() 
                             << ", rtprop is " << rtprop_.get() << endl;
  }
  
  update_machine(timestamp_ms);
  clear_expired_packets(timestamp_ms);
  post_updates();
}
