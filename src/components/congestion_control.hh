#ifndef CONGESTION_CONTROL_HH
#define CONGESTION_CONTROL_HH

#include <memory>
#include <mutex>
#include <map>
#include <set>
#include <vector>
#include "packet.hh"  

class CongestionControlObserver 
{
public:
  virtual void post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps, double loss_rate) = 0;
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
  virtual void on_observer_added(ObserverPtr & obs) { obs->post_updates(1000*125, 800*125, 0); }
  void add_observer(ObserverPtr obs) { observers_.push_back(obs); this->on_observer_added(obs); }
  virtual ~CongestionControlInterface() = default;
};

/**
 * helper function to calculate number of packets sent and received
 */
class CCStatsRecorder : public CongestionControlObserver
{
private:
  uint32_t last_log_ms_ {0};
  uint32_t log_period_ {1000};
  uint32_t pkt_counter_{0};
  uint32_t size_counter_{0};
  uint32_t ack_counter_{0};
  double moving_delay_{0};
  uint32_t sending_rate_{0};
  uint32_t target_bitrate_{0};
  double loss_rate_{0};
  void trigger_log(uint32_t timestamp_ms);
public:
  CCStatsRecorder(uint32_t log_period_ms = 1000) : log_period_(log_period_ms) {}
  void on_packet_sent(uint32_t timestamp_ms, const Packet &p); 
  void on_ack_received(uint32_t timestamp_ms, const AckPacket &p);
  virtual void post_updates(uint32_t sending_rate_byteps, uint32_t target_bitrate_byteps, double loss_rate) 
  {
    sending_rate_ = sending_rate_byteps;
    target_bitrate_ = target_bitrate_byteps;
    loss_rate_ = loss_rate;
  }
};

/**
 * Helper class to calculate the loss rate based on packets sent and ack received
 */
class LossCalculator
{
private:
  constexpr static int LOSS_INTERVAL_MS = 1000;
  using SeqNum = std::tuple<uint32_t, uint32_t, uint16_t>; // send_time_ms, frame_no, frag_no

private:
  std::set<SeqNum> acked_ {};
  std::set<SeqNum> unacked_ {}; // seqnum to timestamp
  uint32_t latest_ack_timestamp_ms_ {0};

private:
  SeqNum get_seq(const Packet & p) { return {p.send_timestamp_ms(), p.frame_no(), p.fragment_no()}; }
  SeqNum get_seq(const AckPacket & p) { return {p.send_time_ms(), p.frame_no(), p.fragment_no()}; }

public:
  void on_packet_sent(uint32_t timestamp_ms, const Packet &p);
  void on_ack_received(uint32_t timestamp_ms, const AckPacket &p);

  /**
   * start calculate from the latest ack - LOSS_INTERVAL_MS to the latest ack
   */
  double get_loss_rate(); // real loss rate = return value / 65535.0f
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

/**
 * class OracleCongestionControl
 * implements the oracle congestion control
 * the rate will be controlled by the bandwidth controller in another thread
 */
class OracleCongestionControl : public CongestionControlInterface
{
private:
  std::mutex mut_ {};
  uint32_t real_bw_byteps_ {0};
  double real_loss_rate_ {0};

  std::shared_ptr<CCStatsRecorder> stats_{};

private:
  void post_updates(); 

public:
  OracleCongestionControl() { stats_ = std::make_shared<CCStatsRecorder>(); add_observer(stats_); }

  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet &p) override;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket &p) override;

  void set_bw(uint32_t bw_byteps);
  void set_loss(double loss_rate);

  uint32_t get_bw();
  double get_loss();
};

/**
 * class SalsifyCongestionControl:
 * implements the congestion control from salsify's paper
 */
class SalsifyCongestionControl : public CongestionControlInterface
{
private:
  constexpr static uint32_t MIN_RATE_BYTEPS = 50 * 125; // 50 kbps
  static constexpr double ALPHA = 0.1;
  static constexpr double MTU = 1400;
  using SeqNum = std::tuple<uint32_t, uint32_t, uint16_t>; // send_time_ms, frame_no, frag_no
  
private:
  // for calculating the number of packets: D / tao_i - Ni
  uint32_t d_ms_ {100};
  double tao_ms_ {-1};
  std::set<SeqNum> inflight_{};

  double ewma_rtt_ms_ {0};

  uint32_t last_recv_ms_ {0};
  std::map<uint32_t, uint32_t> grace_periods_ {}; // key: frame_id (latter frame), value: grace_period in ms

  uint32_t curr_frame_id_ {0};
  int32_t last_sent_ms_ {-1}; // to compute grace_period

  uint16_t fps_ {0};

  std::shared_ptr<CCStatsRecorder> stats_{};
  LossCalculator loss_calc_ {};

private:
  void update_grace_period(uint32_t frame_id, uint32_t value_ms);
  uint32_t query_grace_period(uint32_t frame_id);
  void update_estimation(uint32_t recv_timestamp_ms, uint32_t grace_period_ms, uint32_t pkt_size);
  void post_updates();
  void remove_expired_packets(uint32_t now_ms);

  SeqNum get_seq(const Packet & p) { return {p.send_timestamp_ms(), p.frame_no(), p.fragment_no()}; }
  SeqNum get_seq(const AckPacket & p) { return {p.send_time_ms(), p.frame_no(), p.fragment_no()}; }

public:
  SalsifyCongestionControl(uint32_t D_ms, uint16_t fps);
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet &p) override;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket &p) override;

  /* setter */
  void set_fps(uint16_t fps) { fps_ = fps; }
};


/**
 * class GCCMinus
 * A simplified version of GCC
 */
class GCCMinus : public CongestionControlInterface
{
public:
  enum RateControlState {HOLD = 0, INCREASE=1, DECREASE=2};

public:

  /**
   * calculate the receiving rate per UPDATE_INTERVAL_MS ms
   * the rate is averaged among the latest WINDOW_MS ms
   */
  class RateEstimator
  {
    private:
      constexpr static uint32_t WINDOW_MS = 300; // calculate avg recv rate in latest XXX ms
      constexpr static uint32_t UPDATE_INTERVAL_MS = 50; // update every 50 ms
      
      using SeqNum = std::pair<uint32_t, uint16_t>; // frame_no, frag_no

    private:
      std::map<SeqNum, Packet> unacked_ {};
      std::multimap<uint32_t, uint32_t> acked_bytes_ {};  // acked bytes in the last window

      uint32_t estimate_rate_byteps_ {100 * 125}; // default 100kbps
      uint32_t last_update_ms_ {0};
    public:
      void on_packet_sent(uint32_t timestamp_ms, const Packet & pkt);
      void on_ack_received(uint32_t timestamp_ms, const AckPacket & pkt);

      uint32_t get_rate_estimation() const { return estimate_rate_byteps_; }
  };

  /**
   * calculate the delay related values: d_send, d_recv, d_size for each frame
   */
  class DelayEstimator
  {
    private:
      struct FrameInfo 
      {
        bool send_completed {false};
        bool recv_completed {false};
        uint32_t t_last {0}, T_last {0};
        uint32_t total_pkts = 0;
        uint32_t total_bytes = 0;
      };

    public:
      struct DelayResult
      {
        uint32_t d_send {0};
        uint32_t d_recv {0};
        uint32_t d_size {0};
        bool valid {false};
      };

    private:
      std::map<uint32_t, FrameInfo> frames_ {};
      uint32_t curr_completed_frame_ {0};

      DelayResult result_ {};
  
    private:
      void new_complete_frame(uint32_t frame_no);
    
    public:
      void on_packet_sent(uint32_t timestamp_ms, const Packet & pkt);
      void on_ack_received(uint32_t timestamp_ms, const AckPacket & pkt);

      const DelayResult & get_result() const { return result_; }
  };

  /**
   * use d_send, d_recv and d_size to compute the m_i
   */
  class TrendlineFilter
  {
    private:
      constexpr static double ALPHA = 0.2;
      bool initialized_ { false };
      double m_i_ {-1};
    public:
      void update(const DelayEstimator::DelayResult & delay_result);
      double get_estimation() const; // get m_i
  };

  /**
   * use m_i to compute gamma_i, and then derive the current state
   */
  class OveruseDetector
  {
    private:
      enum Signal {NORMAL = 0, UNDERUSE = -1, OVERUSE = 1};
      constexpr static double GAMMA = 1;
    private:
      RateControlState state_;
    
      void signal_state_change(Signal signal); 
    public:
      void update(double m_i);
      RateControlState get_state() const {return state_;}
  };

  /**
   * use state, incoming rate and loss rate together to determine the final rate
   */
  class AimdController
  {
    private: 
      constexpr static double LOW_LOSS_THRESH = 0.02;
      constexpr static double HIGH_LOSS_THRESH = 0.10;

      constexpr static uint32_t TRIGGER_INTERVAL_MS = 500;

      constexpr static uint32_t MIN_RATE_BYTEPS = 10 * 125; // 10 kbps
    private:
      uint32_t latest_rate_estimation_byteps_ {800 * 125}; // start from 100Kbps
      uint32_t last_rate_update_ms_ {0};

      double curr_loss_rate_ {0.};
      RateControlState curr_state_ {HOLD};
      uint32_t incoming_rate_byteps_ {0};

    private:
      void update_estimation(uint32_t now_ms); 

    public:
      // TODO: add heuristics in aimd_rate_control.cc in webrtc repo
      // update loss 
      void update_loss(uint32_t timestamp_ms, double loss_rate);

      // update state 
      void update_state(uint32_t timestamp_ms, RateControlState state);

      // update incoming rate
      void update_incoming_rate(uint32_t timestamp_ms, uint32_t incoming_rate_byteps);
      
      // get the final rate
      uint32_t get_rate_estimation(uint32_t now_ms);

      /**
       * set the initial rate of the AIMD controller
       */
      void set_rate_estimation(uint32_t rate_byteps) { latest_rate_estimation_byteps_ = rate_byteps; }
  };


private:
  constexpr static uint32_t UPDATE_INTERVAL_MS = 100;

private:
  RateEstimator rate_est_ {};
  DelayEstimator delay_est_ {};
  TrendlineFilter trendline_ {};
  OveruseDetector overuse_ {};
  AimdController aimd_ {};
  LossCalculator loss_calc_ {};

  std::shared_ptr<CCStatsRecorder> stats_{};

  uint32_t last_update_ms_ {0};
private:
  void post_updates(uint32_t now_ms);

  void internal_update(uint32_t now_ms);

public:
  GCCMinus();

  // main interface
  virtual void on_packet_sent(uint32_t timestamp_ms, const Packet &p) override;
  virtual void on_ack_received(uint32_t timestamp_ms, const AckPacket &p) override;

  void set_initial_rate_estimation(uint32_t rate_byteps) { aimd_.set_rate_estimation(rate_byteps); }

  ~GCCMinus() = default;
};

#endif
