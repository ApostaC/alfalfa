#include <unistd.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <iostream>
#include <fstream>

#include "transmission.hh"
#include "comp_encoder.hh"
#include "frame_observer.hh"
#include "timestamp.hh"
#include "bw_control.hh"

using namespace std; 

namespace {
  shared_ptr<CompleteFrameObserver> encode_time_recorder {};
  string output_csv {};
} // anonymous namespace for gingressbal variables

void print_usage(char *argv[])
{
  cerr << "Usage: " << argv[0] << " <ip> <port> <fec rate> <output file>" << endl;
}

void dump_output_time()
{
  ofstream fout(output_csv, ios::out);
  if (not fout or not encode_time_recorder) {
    cerr << "Errorain dump_output_time" << endl; 
  }

  encode_time_recorder->to_csv(fout);
  fout.close();
}

void sigint_handler(int signum) 
{
  cerr << "Catch SIGINT!" << endl;
  dump_output_time();
  int err = system("sudo tc -s qdisc ls dev ingress"); // show tc status before exit
  err = system("sudo tc qdisc del dev ingress root");
  (void)err;
  exit(signum);
}

int main(int argc, char *argv[])
{
  std::cerr.sync_with_stdio(false);
  signal(SIGINT, sigint_handler);
  output_csv = "test-sender.csv";

  if (argc != 5) {
    print_usage(argv);
    exit(1);
  }
  output_csv = argv[4];

  // start TC
  int err;
  err = system("sudo tc qdisc del dev ingress root");
  err = system("sudo tc qdisc add dev ingress root handle 1: tbf rate 5000kbit buffer 1500 latency 300ms");
  if (err) {
    throw runtime_error("Cannot start TC tbf");
  }
  //err = system("sudo tc qdisc add dev ingress parent 1:1 handle 10: netem loss random 5%");
  //if (err) {
  //  throw runtime_error("Cannot start TC netem");
  //}

  BandwidthController bw_ctrl("../../test/test-loss.csv", "ingress", 300);

  int fps = 25;
  BasicEncoder encoder(500 * 125, fps);
  encoder.set_protection_overhead(std::stod(argv[3]));
  //double fec_rate = std::stod(argv[3]);
  //encoder.set_fec_rate(255u * fec_rate);

  //SalsifyCongestionControl cc(100, fps);
  GCCMinus cc;
  cc.set_initial_rate_estimation(bw_ctrl.init_bandwidth_byteps() * 0.6);
  OracleCongestionControl & orac_cc = bw_ctrl.get_oracle_cc();

  RTXManager rtx_mgr;

  encode_time_recorder = std::make_shared<CompleteFrameObserver>();
  encoder.add_frame_observer(encode_time_recorder);

  auto sender = std::make_shared<TransSender>(Address(argv[1], argv[2]), fps, std::ref(cc), std::ref(encoder), std::ref(rtx_mgr));
  cc.add_observer(sender);
  //auto sender = std::make_shared<TransSender>(Address(argv[1], argv[2]), fps, std::ref(orac_cc), std::ref(encoder), std::ref(rtx_mgr));
  //orac_cc.add_observer(sender);

  cout << "Starting sender!" << endl;
  uint32_t limit_ms = 12 * 1000;
  bw_ctrl.start();
  sender->start(limit_ms);

  bw_ctrl.stop();
  dump_output_time();
  (void)orac_cc;
  return 0;
}
