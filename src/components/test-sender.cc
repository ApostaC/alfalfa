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
#include "stats.hh"

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

RTXInterface & get_rtx(const std::string & name)
{
  static RTXManager all_rtx;
  static NoRTX no_rtx;
  static SVCRTXManager svc_rtx;
  if (name == "all") {
    cerr << "Using all RTX" << endl;
    return std::ref(all_rtx);
  }
  else if (name == "svc") {
    cerr << "Using svc RTX" << endl;
    return std::ref(svc_rtx);
  }
  else {
    cerr << "Using no rtx" << endl;
    return std::ref(no_rtx);
  }
}

CongestionControlInterface & get_cc(const std::string & name, BandwidthController & bw, int fps=25)
{
  static SalsifyCongestionControl cc(100, fps);
  static GCCMinus gcc; gcc.set_initial_rate_estimation(bw.init_bandwidth_byteps() * 0.6);
  static BBRMinus bbr; bbr.set_initial_rate_estimation(bw.init_bandwidth_byteps() * 0.6);
  if (name == "salsify") {
    return std::ref(cc);
  }
  else if (name == "gcc") {
    return std::ref(gcc);
  }
  else if (name == "bbr") {
    return std::ref(bbr);
  }
  else {
    return bw.get_oracle_cc();
  }
}

EncoderInterface & get_codec(const std::string & name, double prot, int fps=25)
{
  static BasicEncoder basic_enc(500 * 125, fps);
  basic_enc.set_protection_overhead(prot);
  static SVCEncoder svc_enc(500 * 125, fps, {0.4, 0.3, 0.3}, {255, 0, 0});
  
  if (name == "svc") {
    return std::ref(svc_enc);
  }
  else {
    return std::ref(basic_enc);
  }
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

  BandwidthController bw_ctrl("../../test/fcc_for_emulation/0.csv", "ingress", 300);
  //BandwidthController bw_ctrl("../../test/test-bw.csv", "ingress", 300);

  int fps = 25;
  auto & encoder = get_codec("basic", std::stod(argv[3]), fps);
  auto & cc = get_cc("bbr", bw_ctrl, fps);
  auto & rtx_mgr = get_rtx("all");
  //BasicEncoder encoder(500 * 125, fps);
  //encoder.set_protection_overhead(std::stod(argv[3]));
  //double fec_rate = std::stod(argv[3]);
  //encoder.set_fec_rate(255u * fec_rate);

  //SalsifyCongestionControl cc(100, fps);
  //GCCMinus cc;
  //BBRMinus cc;
  //cc.set_initial_rate_estimation(bw_ctrl.init_bandwidth_byteps() * 0.6);
  //OracleCongestionControl & orac_cc = bw_ctrl.get_oracle_cc();

  //RTXManager rtx_mgr;

  encode_time_recorder = std::make_shared<CompleteFrameObserver>();
  encoder.add_frame_observer(encode_time_recorder);

  auto real_data = std::make_shared<StatsRecorder>("temp/real.csv");
  auto pred_data = std::make_shared<StatsRecorder>("temp/pred.csv");
  bw_ctrl.get_oracle_cc().add_observer(real_data);
  cc.add_observer(pred_data);

  auto sender = std::make_shared<TransSender>(Address(argv[1], argv[2]), fps, std::ref(cc), std::ref(encoder), std::ref(rtx_mgr));
  cc.add_observer(sender);
  //auto sender = std::make_shared<TransSender>(Address(argv[1], argv[2]), fps, std::ref(orac_cc), std::ref(encoder), std::ref(rtx_mgr));
  //orac_cc.add_observer(sender);

  cout << "Starting sender!" << endl;
  uint32_t limit_ms = 30 * 1000;
  bw_ctrl.start();
  sender->start(limit_ms);

  bw_ctrl.stop();

  real_data->dump(true);
  pred_data->dump(true);
  dump_output_time();
  return 0;
}
