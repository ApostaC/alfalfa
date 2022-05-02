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

using namespace std; 

namespace {
  shared_ptr<CompleteFrameObserver> encode_time_recorder {};
  string output_csv {};
} // anonymous namespace for gingressbal variables

void print_usage(char *argv[])
{
  cerr << "Usage: " << argv[0] << " <ip> <port> <fec rate>" << endl;
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

  if (argc != 4) {
    print_usage(argv);
    exit(1);
  }

  // start TC
  int err;
  err = system("sudo tc qdisc del dev ingress root");
  err = system("sudo tc qdisc add dev ingress root handle 1: tbf rate 1000kbit buffer 1500 latency 300ms");
  if (err) {
    throw runtime_error("Cannot start TC tbf");
  }
  //err = system("sudo tc qdisc add dev ingress parent 1:1 handle 10: netem loss random 5%");
  //if (err) {
  //  throw runtime_error("Cannot start TC netem");
  //}


  int fps = 25;
  BasicEncoder encoder(500 * 125, fps);
  double fec_rate = std::stod(argv[3]);
  encoder.set_fec_rate(255u * fec_rate);
  SalsifyCongestionControl cc(100, fps);
  //GCCMinus cc;
  RTXManager rtx_mgr;

  encode_time_recorder = std::make_shared<CompleteFrameObserver>();
  encoder.add_frame_observer(encode_time_recorder);

  auto sender = std::make_shared<TransSender>(Address(argv[1], argv[2]), fps, std::ref(cc), std::ref(encoder), std::ref(rtx_mgr));
  cc.add_observer(sender);

  // start a thread throttoles the bandwidth at 5 sec
  //using namespace std::chrono_literals;
  //thread tc_change([](){
  //      this_thread::sleep_for(5s);
  //      int err = system("sudo tc qdisc change dev ingress root handle 1: tbf rate 300kbit buffer 1500 limit 10000");
  //      if (err != 0) {
  //        cerr << "Warning: channot change TC bitrate control!" << endl;
  //      }
  //      return;
  //    });

  cout << "Starting sender!" << endl;
  uint32_t limit_ms = 12 * 1000;
  sender->start(limit_ms);
  dump_output_time();

  //tc_change.join();
  err = system("sudo tc -s qdisc ls dev ingress"); // show tc status before exit
  err = system("sudo tc qdisc del dev ingress root");
  return 0;
}
