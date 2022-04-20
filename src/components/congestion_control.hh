#ifndef CONGESTION_CONTROL_HH
#define CONGESTION_CONTROL_HH

#include <memory>
#include <vector>
#include "packet.hh"  

class CongestionControlObserver 
{
public:
  virtual void post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps) = 0;
};

class CongestionControlInterface 
{
private:
  using ObserverPtr = std::shared_ptr<CongestionControlObserver>;
  std::vector<ObserverPtr> observers_;

public:
  // main interface
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet &p) = 0;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket &p) = 0;

  void add_observer(ObserverPtr obs) { observers_.push_back(obs); }
  virtual ~CongestionControlInterface() = default;
};

#endif
