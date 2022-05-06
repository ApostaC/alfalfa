#ifndef BW_CONTROL_HH
#define BW_CONTROL_HH

#include <cstdlib>
#include <string>
#include <functional>
#include <thread>
#include <cstdio>

#include "optional.hh"
#include "congestion_control.hh"


/**
 * Controls the bandwidth using TC-tbf, do not control the delay and loss
 * After calling start(), a new thread will be started to control the bandwidth.
 * When the thread wakes up, it will check the time and see if there is any updates.
 *
 * There are 2 types of updates:
 *  1. update the TC by the trace. The triggering condition is: curr time falls into
 *     the next period
 *  2. update the oracle CC by the trace. Oracle CC should going down a bit earlier 
 *     before trace bw drops, and going up a bit later than trace bw grows.
 *
 * It wil load the trace from a file, the format of a trace is:
 *    <offset_ms> <bw_kbit/s> (delimited by space)
 * The offset of the first line should always be 0
 */
class BandwidthController
{
private:
  struct TCOperation
  {
    uint32_t target_bw_kbitps;
    double loss_rate;
    bool initialize;
    BandwidthController *ptr;

    TCOperation(uint32_t tgt_bw_kbitps, double loss, bool init, BandwidthController *p)
      : target_bw_kbitps(tgt_bw_kbitps), loss_rate(loss), initialize(init), ptr(p) {}

    void operator()() { ptr->update_rules(target_bw_kbitps, loss_rate, initialize); }
  };

  struct CCOperation
  {
    uint32_t target_bw_byteps;
    double loss;
    BandwidthController *ptr;

    CCOperation(uint32_t tgt_bw_byteps, double l, BandwidthController *p)
      : target_bw_byteps(tgt_bw_byteps), loss(l), ptr(p) {}

    void operator()() { ptr->oracle_cc_.set_bw(target_bw_byteps); ptr->oracle_cc_.set_loss(loss); }
  };

  struct Event
  {
    uint32_t timestamp_ms;
    std::function<void()> callback;
  };

private:
  const std::string dev_;

  std::vector<Event> events_ {};

  /* main thread */
  //Optional<std::thread> worker_ {};
  std::thread worker_{};

  /* the oracle cc */
  OracleCongestionControl oracle_cc_ {};

  /* queue length and loss */
  uint32_t queue_len_ms_;

  std::mutex mut_ {};
  bool is_running_ {false};

  uint32_t init_bw_byteps_ {};

private:
  /**
   * run the system() command line, and show error msg if there is any error
   */
  static void RunCmd(const std::string & cmd, const std::string & err_msg, bool strict=true);

  /* tc related operations */
  void delete_rules();
  void update_rules(uint32_t bw_kbitps, double loss_rate, bool initialize = false);
  void show_status();

  /* load the trace and update the event times */
  void load_trace(const std::string & trace_file);

  void set_running(bool running);
  bool get_running();

public:
  /**
   * trace_file: the trace file whose format is <offset_ms> <bw_kbit/s>
   * dev: the name of the network device
   * queue_len_ms: the tbf queue length (see `latency' in tbf man page)
   * loss rate: the random loss rate of the link
   */
  BandwidthController(const std::string & trace_file, const std::string & dev, 
                      const uint32_t queue_len_ms);

  OracleCongestionControl & get_oracle_cc() { return oracle_cc_; }
  void start();
  void stop();

  uint32_t init_bandwidth_byteps() const { return init_bw_byteps_; }

  ~BandwidthController();
};

#endif
