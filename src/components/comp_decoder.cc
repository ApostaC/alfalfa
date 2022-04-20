#include <iostream>
#include "comp_decoder.hh"

using namespace std;

void NonBlockingDecoder::flush_frame(uint32_t timestamp_ms, const FragmentedFrame & frame)
{
  for(auto obs : frame_observers_) {
    if (frame.complete()) {
      obs->new_complete_frame(timestamp_ms, frame);
    }
    else {
      obs->new_incomplete_frame(timestamp_ms, frame);
    }
  }
}

void NonBlockingDecoder::incoming_packet(uint32_t timestamp_ms, const Packet & pkt)
{
  // from a early frame, skip
  if (pkt.frame_no() < expected_frame_no_) {
    //cerr << "Received a packet for old frame " << pkt.frame_no() << ", skip!" << endl;
    return;
  }

  // for a future frame, push all the incomplete frames into observers
  if (pkt.frame_no() > expected_frame_no_) {
    cerr << "Received a packet for future frame " << pkt.frame_no() 
         << ", flushing previous frames" << endl;

    // flush old frames
    for(auto i = expected_frame_no_; i < pkt.frame_no(); ++i) {
      if(frames_.count(i) == 0) continue;
      flush_frame(timestamp_ms, frames_.at(i));
      frames_.erase(i);
    }

    // update expected_frame_no_
    expected_frame_no_ = pkt.frame_no();
  }
  
  // it's the current frame, add the packet and check if it's complete
  if (frames_.count(pkt.frame_no())) {
    frames_.at(pkt.frame_no()).add_packet(pkt);
  }
  else {
    frames_.insert(std::make_pair<uint32_t, FragmentedFrame>(pkt.frame_no(), {0, pkt}));
  }

  assert(frames_.count(pkt.frame_no()) > 0);

  // it's complete, flush the frame and update the state
  auto & frame = frames_.at(pkt.frame_no());
  if (frame.complete()) {
    flush_frame(timestamp_ms, frame);
    frames_.erase(pkt.frame_no());
    expected_frame_no_ += 1;
  }
  
}
