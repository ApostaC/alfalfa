#include "frame_observer.hh"

using namespace std;

void FrameArrivalTimeObserver::new_complete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame)
{
  if (arrival_time_.count(frame.frame_no()) == 0) {
    arrival_time_[frame.frame_no()] = timestamp_ms;
  }
  else {
    throw runtime_error("FrameArrivalTimeObserver: adding a duplicate frame whose id = " + to_string(frame.frame_no()));
  }
}

void FrameArrivalTimeObserver::new_incomplete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame)
{
  new_complete_frame(timestamp_ms, frame);
}

void CompleteFrameObserver::new_complete_frame(uint32_t timestamp_ms, const FragmentedFrame & frame)
{
  if (frames_.count(frame.frame_no()) == 0) {
    FrameInfo info{frame.frame_no(), timestamp_ms, (uint32_t)frame.frame().length()};
    frames_[frame.frame_no()] = info;
  }
  else {
    throw runtime_error("CompleteFrameObserver: adding a duplicate frame whose id = " + to_string(frame.frame_no()));
  }
}

void CompleteFrameObserver::new_incomplete_frame(uint32_t, const FragmentedFrame &)
{
  throw runtime_error("CompleteFrameObserver: only supports complete frames!");
}

void CompleteFrameObserver::to_csv(ostream & out)
{
  for(auto & info : frames_) {
    out << info.first << "," << info.second.arrival_time
        << "," << info.second.size_in_bytes << endl;
  }
}
