#ifndef TRANSMISSION_HH
#define TRANSMISSION_HH

#include "timerfd.hh"
#include "poller.hh"
#include "socket.hh"
#include "congestion_control.hh"
#include "comp_encoder.hh"
#include "comp_decoder.hh"

class RTXInterface
{
public:
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet & pkt) = 0;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket & ack, std::deque<Packet> & target_buf) = 0;
  virtual void on_rtx_sent(uint32_t timestamp_ms, const Packet & pkt) = 0;
  virtual uint32_t get_rtx_bitrate_byteps(uint32_t timestamp_ms) = 0;

  virtual ~RTXInterface() = default;
};

class NoRTX : public RTXInterface
{
  virtual void on_packet_sent(uint32_t, const Packet & ) override {}
  virtual void on_ack_received(uint32_t, const AckPacket &, std::deque<Packet> &) override {};
  virtual void on_rtx_sent(uint32_t, const Packet &) override {}
  virtual uint32_t get_rtx_bitrate_byteps(uint32_t) override { return 0; }
};

class RTXManager : public RTXInterface
{
private:
  using SeqNum = std::pair<uint32_t, uint16_t>; // frame_no, frag_no
  struct UnackedInfo 
  {
    Packet packet {};
    uint32_t num_rtx = 0;
    uint32_t last_sent_ms = 0;
    bool is_waiting = false;
  };

  std::map<SeqNum, UnackedInfo> unacked_ {};    // unack queue
  std::map<SeqNum, uint32_t> num_rtx_ {};       // number of retransmissions per unacked packet
  std::map<SeqNum, uint32_t> last_sent_ms_ {};  // time of RTX pkt sent out

  Optional<uint32_t> ewma_rtt_ms_ {};           // rtt estimation
  constexpr static double ALPHA = 0.2;

  std::map<uint32_t, uint32_t> sent_rtx_size_ {}; // RTX rate estimation: <timestamp_ms, size_in_bytes>
  constexpr static uint32_t RTX_RATE_WINDOW_MS = 500;

private:
  void add_unacked(const Packet & pkt);
  void add_rtt_sample(uint32_t rtt_ms);

public:
  RTXManager();
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet & pkt) override;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket & ack, std::deque<Packet> & tgt_buf) override;
  virtual void on_rtx_sent(uint32_t timestamp_ms, const Packet & pkt) override;
  virtual uint32_t get_rtx_bitrate_byteps(uint32_t timestamp_ms) override; 
};

/**
 * The pacer with different prioirty queues
 */
class BudgetPacer
{
private:
  constexpr static int DEFAULT_MAX_BUDGET = 500;

private:
  uint32_t last_update_timestamp_ms_ {0};
  uint32_t pacing_rate_byteps_ {0};
  int budget_ {0};
  int max_budget_ {DEFAULT_MAX_BUDGET};

private:
  void update_budget(uint32_t timestamp_ms);

public:
  BudgetPacer(int max_budget_ = 500);

  // start the pacer
  void start_pacer(uint32_t timestamp_ms) { last_update_timestamp_ms_ = timestamp_ms; }

  // set pacing rate
  void set_pacing_rate(uint32_t rate_byteps);

  // called when sending out a packet 
  void on_packet_sent(uint32_t timestamp_ms, size_t pkt_size);

  // how long should we wait until ready
  uint32_t ms_until_nextcheck() const 
  { 
    if (pacing_rate_byteps_ > 0) {
      return std::min(budget_ * 1000 / pacing_rate_byteps_, 10u);
    }
    return 10;
  }

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
  constexpr static uint32_t INITIAL_PACING_RATE = 100 * 125;

private:
  Poller poller_ {};
  UDPSocket socket_{};
  Timerfd frame_timer_ {};
  BudgetPacer pacer_ {};
  CongestionControlInterface & cc_;
  EncoderInterface & encoder_;
  RTXInterface & rtx_mgr_;

  uint32_t cached_pacing_rate_ {INITIAL_PACING_RATE};

  std::deque<Packet> data_queue_{};
  std::deque<Packet> rtx_queue_{};

private:
  bool has_data_to_send() const { return not (data_queue_.empty() and rtx_queue_.empty()); }
  void send_one_packet(uint32_t now_ms);
  void send_stop_message();

public:
  TransSender(const Address & peer_addr, uint32_t fps, 
              CongestionControlInterface & cc, 
              EncoderInterface & encoder,
              RTXInterface & rtx_mgr);

  // start the main loop
  void start(uint32_t time_limt_ms = -1);

  // implements CongestionControlObserver
  void post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps, double loss_rate) override; 
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

  bool stop_received_ {false};
  
public:
  TransReceiver(const uint16_t port, DecoderInterface & decoder);

  // start the main loop
  void start(uint32_t time_limit_ms = -1);
};
#endif
