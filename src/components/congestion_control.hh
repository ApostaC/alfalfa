#ifndef CONGESTION_CONTROL_HH
#define CONGESTION_CONTROL_HH

#include <memory>
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

#endif
