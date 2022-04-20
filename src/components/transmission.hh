#ifndef TRANSMISSION_HH
#define TRANSMISSION_HH

#include "congestion_control.hh"

class TransSender 
    : public CongestionControlObserver
{
public:
  void post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps) override;
};

#endif
