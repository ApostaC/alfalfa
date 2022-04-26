#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <fstream>

#include "transmission.hh"
#include "comp_decoder.hh"
#include "frame_observer.hh"

using namespace std; 

namespace {
  shared_ptr<CompleteFrameObserver> decode_time_recorder {};
  string output_csv {};
} // anonymous namespace for global variables

void print_usage(char *argv[])
{
  cerr << "Usage: " << argv[0] << " <port>" << endl;
}

void dump_output_time()
{
  ofstream fout(output_csv, ios::out);
  if (not fout or not decode_time_recorder) {
    cerr << "Errorain dump_output_time" << endl; 
  }

  decode_time_recorder->to_csv(fout);
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
  output_csv = "test-receiver.csv";

  if (argc != 2) {
    print_usage(argv);
    exit(1);
  }

  BlockingDecoder decoder;
  RTXManager rtx_mgr;

  decode_time_recorder = std::make_shared<CompleteFrameObserver>();
  decoder.add_frame_observer(decode_time_recorder);

  uint16_t port = std::stoul(argv[1]);
  auto receiver = std::make_shared<TransReceiver>(port, decoder);
  
  cout << "Starting receiver!" << endl;
  receiver->start();
  return 0;
}
