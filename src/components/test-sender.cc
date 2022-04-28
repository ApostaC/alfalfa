#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <fstream>

#include "transmission.hh"
#include "comp_encoder.hh"
#include "frame_observer.hh"

using namespace std; 

namespace {
  shared_ptr<CompleteFrameObserver> encode_time_recorder {};
  string output_csv {};
} // anonymous namespace for global variables

void print_usage(char *argv[])
{
  cerr << "Usage: " << argv[0] << " <ip> <port>" << endl;
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
  exit(signum);
}

int main(int argc, char *argv[])
{
  signal(SIGINT, sigint_handler);
  output_csv = "test-sender.csv";

  if (argc != 3) {
    print_usage(argv);
    exit(1);
  }

  int fps = 25;
  BasicEncoder encoder(500 * 125, fps);
  encoder.set_fec_rate(255u * 0.3);
  //SalsifyCongestionControl cc(100, fps);
  GCCMinus cc;
  RTXManager rtx_mgr;

  encode_time_recorder = std::make_shared<CompleteFrameObserver>();
  encoder.add_frame_observer(encode_time_recorder);

  auto sender = std::make_shared<TransSender>(Address(argv[1], argv[2]), fps, std::ref(cc), std::ref(encoder), std::ref(rtx_mgr));
  cc.add_observer(sender);

  cout << "Starting sender!" << endl;
  uint32_t limit_ms = 12 * 1000;
  sender->start(limit_ms);
  dump_output_time();
  return 0;
}
