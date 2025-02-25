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

void BlockingDecoder::on_frame_complete(uint32_t timestamp_ms, const FragmentedFrame & frame)
{
  // erase all the previous partial frames
  auto frame_no = frame.frame_no();
  for(auto i = already_decoded_no_; i < frame_no; i++) {
    frames_.erase(i);
  }

  // update state
  already_decoded_no_ = frame.frame_no();

  // update observers
  for (auto obs : frame_observers_) {
    obs->new_complete_frame(timestamp_ms, frame);
  }

  // remove this frame from the queue
  frames_.erase(frame_no);
}

void BlockingDecoder::incoming_packet(uint32_t timestamp_ms, const Packet & pkt)
{
  auto frame_no = pkt.frame_no();

  // an outdated packet is received, skip
  if (frame_no <= already_decoded_no_) {
    return;
  }

  // received packets in an exising frame
  if (frames_.count(frame_no)) {
    frames_.at(frame_no).add_packet(pkt);
  }
  else { // received packets in a new frame
    frames_.insert(std::make_pair<uint32_t, FragmentedFrame>(pkt.frame_no(), {0, pkt}));
  }

  // check if a key frame is decodable
  for (auto & frame : frames_) {
    if (frame.second.is_key_frame() and frame.second.complete()) {
      on_frame_complete(timestamp_ms, frame.second);
    }
  }

  // check if the next frame is decodable
  auto expected_frame_no = already_decoded_no_ + 1;
  while (frames_.count(expected_frame_no) and frames_.at(expected_frame_no).complete()) {
    auto & frame = frames_.at(expected_frame_no);
    on_frame_complete(timestamp_ms, frame);
    expected_frame_no = already_decoded_no_ + 1;
  }
}
