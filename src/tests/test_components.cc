#include <iostream>
#include "comp_encoder.hh"
#include "comp_decoder.hh"
#include "frame_observer.hh"

using namespace std; 

ostream & out_green(ostream & out)
{
  out << "\033[32m";
  return out;
}

ostream & out_red(ostream & out)
{
  out << "\033[31m";
  return out;
}

ostream & out_yellow(ostream & out)
{
  out << "\033[33m";
  return out;
}

ostream & out_normal(ostream & out)
{
  out << "\033[0m";
  return out;
}

bool test_basic_encoder_decoder()
{
  out_green(cerr) << "============== test: basic encoder decoder ================="; out_normal(cerr) << endl;
  BasicEncoder encoder(500 * 125, 25);
  NonBlockingDecoder decoder;
  auto frame_obs = std::make_shared<FrameArrivalTimeObserver>();
  decoder.add_frame_observer(frame_obs);

  auto frame1 = encoder.encode_next_frame(0);
  encoder.set_target_bitrate(1000 * 125);
  auto frame2 = encoder.encode_next_frame(40);

  assert(frame1.frame_no() == 1);
  assert(frame2.frame_no() == 2);

  cerr << "Frame1: " << frame1.packets().size() << " packets" << endl;
  cerr << "Frame2: " << frame2.packets().size() << " packets" << endl;

  for (auto & pkt : frame1.packets()) {
    decoder.incoming_packet(pkt.fragment_no(), pkt);
  }

  for (auto & pkt : frame2.packets()) {
    decoder.incoming_packet(40 + pkt.fragment_no(), pkt);
  }
  
  const auto & arrival_time = frame_obs->arrival_time();
  (void)(arrival_time);
  assert(arrival_time.size() == 2);
  assert(arrival_time.at(1) == frame1.packets().size());
  assert(arrival_time.at(2) == frame2.packets().size());
  out_green(cerr) << "-------------- PASSED: basic encoder decoder ----------------"; out_normal(cerr) << endl;
  return true;
}

int main(int argc, char *argv[])
{
  (void)(argc);
  (void)(argv);
  out_green(cerr) << "Starting test!";
  out_normal(cerr) << endl;

  test_basic_encoder_decoder();
}
