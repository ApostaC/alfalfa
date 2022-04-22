#ifndef CONGESTION_CONTROL_HH
#define CONGESTION_CONTROL_HH

#include <memory>
#include <map>
#include <vector>
#include "packet.hh"  

class CongestionControlObserver 
{
public:
  virtual void post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps) = 0;
  virtual ~CongestionControlObserver() = default;
};

class CongestionControlInterface 
{
protected:
  using ObserverPtr = std::shared_ptr<CongestionControlObserver>;
  std::vector<ObserverPtr> observers_ {};

public:
  // main interface
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet &p) = 0;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket &p) = 0;

  // what to do when adding a new observer
  virtual void on_observer_added(ObserverPtr & obs) { obs->post_updates(1000*125, 800*125); }
  void add_observer(ObserverPtr obs) { observers_.push_back(obs); this->on_observer_added(obs); }
  virtual ~CongestionControlInterface() = default;
};

class CCStatsRecorder 
{
private:
  uint32_t last_log_ms_ {0};
  uint32_t log_period_ {1000};
  uint32_t pkt_counter_{0};
  uint32_t size_counter_{0};
  uint32_t ack_counter_{0};
  double moving_delay_{0};
  void trigger_log(uint32_t timestamp_ms);
public:
  CCStatsRecorder(uint32_t log_period_ms = 1000) : log_period_(log_period_ms) {}
  void on_packet_sent(uint32_t timestamp_ms, const Packet &p); 
  void on_ack_received(uint32_t timestamp_ms, const AckPacket &p);
};

class DumbCongestionControl : public CongestionControlInterface
{
private:
  uint32_t last_log_ms_ {0};
  uint32_t log_period_ {1000};
  uint32_t pkt_counter_{0};
  uint32_t size_counter_{0};
  uint32_t ack_counter_{0};
  double moving_delay_{0};
  void trigger_log(uint32_t timestamp_ms);
public:
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet &p) override;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket &p) override;
};

class SalsifyCongestionControl : public CongestionControlInterface
{
private:
  static constexpr double ALPHA = 0.1;
  static constexpr double MTU = 1400;
  
private:
  // for calculating the number of packets: D / tao_i - Ni
  uint32_t d_ms_ {100};
  double tao_ms_ {-1};
  uint32_t ni_ {0};

  uint32_t last_recv_ms_ {0};
  std::map<uint32_t, uint32_t> grace_periods_ {}; // key: frame_id (latter frame), value: grace_period in ms

  uint32_t curr_frame_id_ {0};
  int32_t last_sent_ms_ {-1}; // to compute grace_period

  uint16_t fps_ {0};

  CCStatsRecorder stats_{};
private:
  void update_grace_period(uint32_t frame_id, uint32_t value_ms);
  uint32_t query_grace_period(uint32_t frame_id);
  void update_estimation(uint32_t recv_timestamp_ms, uint32_t grace_period_ms);
  void post_updates();

public:
  SalsifyCongestionControl(uint32_t D_ms, uint16_t fps);
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet &p) override;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket &p) override;

  /* setter */
  void set_fps(uint16_t fps) { fps_ = fps; }
};

#endif
