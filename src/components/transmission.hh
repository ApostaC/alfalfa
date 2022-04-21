#ifndef TRANSMISSION_HH
#define TRANSMISSION_HH

#include "timerfd.hh"
#include "poller.hh"
#include "socket.hh"
#include "congestion_control.hh"
#include "comp_encoder.hh"
#include "comp_decoder.hh"

/**
 * The pacer with different prioirty queues
 */
class BudgetPacer
{
private:
  uint32_t last_update_timestamp_ms_ {0};
  uint32_t pacing_rate_byteps_ {0};
  int budget_ {0};
  int max_budget_ {0};

private:
  void update_budget(uint32_t timestamp_ms);

public:
  BudgetPacer(int max_budget_ = 500);

  // start the pacer
  void start_pacer(uint32_t timestamp_ms) { last_update_timestamp_ms_ = timestamp_ms; }

  // set pacing rate
  void set_pacing_rate(uint32_t rate_byteps) { pacing_rate_byteps_ = rate_byteps; }

  // called when sending out a packet 
  void on_packet_sent(uint32_t timestamp_ms, size_t pkt_size);

  // how long should we wait until ready
  uint32_t ms_until_ready() const { return budget_ * 1000 / pacing_rate_byteps_; }

  // query if it's ready to send
  bool ready_to_send(uint32_t now_ms);
};

/**
 * The real data sender, will drive the timing for all other components
 */
class TransSender 
    : public CongestionControlObserver
{
private:
  Poller poller_ {};
  UDPSocket socket_{};
  Timerfd frame_timer_ {};
  BudgetPacer pacer_ {};
  CongestionControlInterface & cc_;
  EncoderInterface & encoder_;

  std::deque<Packet> data_queue_{};
  std::deque<Packet> rtx_queue_{};

private:
  bool has_data_to_send() const { return not (data_queue_.empty() and rtx_queue_.empty()); }

public:
  TransSender(const Address & peer_addr, uint32_t fps, 
              CongestionControlInterface & cc, 
              EncoderInterface & encoder);

  // start the main loop
  void start();

  // implements CongestionControlObserver
  void post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps) override; 
};


/**
 * receive the packet, notify the decoder and generate acks
 */
class TransReceiver 
{
private:
  Poller poller_ {};
  UDPSocket socket_{};
  DecoderInterface & decoder_;
  
public:
  TransReceiver(const uint16_t port, DecoderInterface & decoder);

  // start the main loop
  void start();
};
#endif
