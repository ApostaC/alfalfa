#include <fstream>
#include "congestion_control.hh"
#include "bw_control.hh"

#include "timestamp.hh"

using namespace std;

BandwidthController::BandwidthController(const std::string & trace_file, const std::string & dev,
                                         const uint32_t queue_len_ms)
  : dev_(dev), oracle_cc_(), queue_len_ms_(queue_len_ms) 
{
  this->delete_rules();
  load_trace(trace_file);
}

void BandwidthController::load_trace(const std::string & trace_file) 
{
  ifstream fin(trace_file);
  uint32_t offset_ms, bw_kbitps;
  double loss;
  int lineno = 0;
  uint32_t last_offset_ms = 0, last_bw_kbitps = 0;
  while(fin >> offset_ms >> bw_kbitps >> loss) {
    if (lineno == 0 and offset_ms != 0) {
      throw runtime_error("BandwidthController::load_trace: first offset must be zero!");
    }

    auto bw_byteps = bw_kbitps * 125;
    if (lineno == 0) {
      /* first line, initialize */
      events_.push_back({offset_ms, CCOperation(bw_byteps, loss, this)});
      events_.push_back({offset_ms, TCOperation(bw_kbitps, loss, true, this)}); // initialize = true
      init_bw_byteps_ = bw_byteps;
    }
    else {
      /* normal routine */
      if (offset_ms - last_offset_ms <= 100) {
        throw runtime_error("BandwidthController::load_trace: bw change too frequent (less than 100ms)");
      }
      
      if (bw_kbitps > last_bw_kbitps) {
        /* bandwidth increase: tc before CC change */
        events_.push_back({offset_ms, TCOperation(bw_kbitps, loss, false, this)}); // initialize = false
        events_.push_back({offset_ms + 50, CCOperation(bw_byteps, loss, this)});
      }
      else {
        /* bandwidth decrease: CC before tc change */
        events_.push_back({offset_ms - 50, CCOperation(bw_byteps, loss, this)});
        events_.push_back({offset_ms, TCOperation(bw_kbitps, loss, false, this)}); // initialize = false
      }
    }

    last_offset_ms = offset_ms;
    last_bw_kbitps = bw_kbitps;
    lineno += 1;
  }
}

void BandwidthController::RunCmd(const string & cmd, const string & err_msg, bool strict)
{
  int err = system(cmd.c_str());
  if (err) {
    cerr << "Error at command: " << cmd << endl;
    if (strict) throw runtime_error("Error: BandwidthController::RunCmd: " + err_msg);
    else cerr << "Warning: BandwidthController::RunCmd: " << err_msg << endl;
  }
}

void BandwidthController::delete_rules()
{
  /* not strict */
  RunCmd("sudo tc qdisc del dev " + dev_ + " root", "failed to delete the existing rules", false);
}

void BandwidthController::update_rules(uint32_t bw_kbitps, double loss_rate, bool initialize)
{
  cerr << "Update rule to: " << bw_kbitps << endl;
  if (initialize) {
    string cmd = "sudo tc qdisc add dev " + this->dev_ + " root handle 1: tbf rate " +
                 to_string(bw_kbitps) + "kbit buffer 1500 latency " + to_string(queue_len_ms_) + "ms";
    RunCmd(cmd, "failed to initialize the TC tbf");
    RunCmd("sudo tc qdisc add dev " + dev_ + " parent 1:1 handle 10: netem loss random " 
            + to_string(loss_rate) + "%", "failed to initialize TC netem");
  }
  else {
    RunCmd("sudo tc qdisc change dev " + dev_ + " root handle 1: tbf rate " + 
           to_string(bw_kbitps) + "kbit buffer 1500 latency " + to_string(queue_len_ms_) + "ms", 
           "failed to change TC tbf rules");
    RunCmd("sudo tc qdisc change dev " + dev_ + " parent 1:1 handle 10: netem loss random " 
            + to_string(loss_rate) + "%", "failed to initialize TC netem");
  }
}

void BandwidthController::show_status()
{
  RunCmd("sudo tc -s qdisc ls dev " + dev_, "failed to display TC status", false);
}

void BandwidthController::set_running(bool running) 
{
  unique_lock<mutex> lock(mut_);
  is_running_ = running;
}

bool BandwidthController::get_running()
{
  unique_lock<mutex> lock(mut_);
  return is_running_;
}

void BandwidthController::start()
{
  using namespace std::chrono_literals;
  set_running(true);
  /* initialize the thread */
  //worker_.initialize([this]()
  worker_ = thread([this]()
      {
        uint32_t start_ms = timestamp_ms();
        uint32_t next_event_id = 0;
        while(true) {
          if (not this->get_running()) break;
          auto now_ms = timestamp_ms();
          while (next_event_id < this->events_.size() and 
                 this->events_.at(next_event_id).timestamp_ms + start_ms <= now_ms) {
            cerr << "[BandwidthController]: Execute an event (" << this->events_.at(next_event_id).timestamp_ms 
                 << ") at " << now_ms - start_ms << endl;
            this->events_.at(next_event_id).callback();
            next_event_id += 1;
          }

          this_thread::sleep_for(10ms);
        }
      });
}

void BandwidthController::stop()
{
  if (not get_running()) {
    return;
  }

  set_running(false);
  if(worker_.joinable()) {
    cerr << "[BandwidthController]: Waiting for worker thread!" << endl;
    worker_.join();
  }
  this->show_status();
  this->delete_rules();
  cerr << "[Bandwidth]: Controller stopped!" << endl;
}

BandwidthController::~BandwidthController() 
{
  stop();
}
