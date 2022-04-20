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
