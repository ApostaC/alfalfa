#ifndef FRAME_OBSERVER_HH
#define FRAME_OBSERVER_HH

#include <map>
#include <iostream>
#include <memory>
#include "packet.hh"

class FrameObserver
{
public:
  virtual void new_complete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame) = 0;
  virtual void new_incomplete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame) = 0;
  
  virtual ~FrameObserver() = default;
};

using FrameObserverPtr = std::shared_ptr<FrameObserver>;

class FrameArrivalTimeObserver
  : public FrameObserver
{
private:
  std::map<uint32_t, uint32_t> arrival_time_ {}; // frame_id -> arrival_time (ms)

public:

  virtual void new_complete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame) override;
  virtual void new_incomplete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame) override;

  const std::map<uint32_t, uint32_t> & arrival_time() const { return arrival_time_; }
};

class CompleteFrameObserver
  : public FrameObserver
{
private:
  struct FrameInfo 
  {
    uint32_t frame_no;
    uint32_t arrival_time;
    uint32_t size_in_bytes;
  };

  std::map<uint32_t, FrameInfo> frames_ {}; // frame_id -> arrival_time (ms)

public:

  virtual void new_complete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame) override;
  virtual void new_incomplete_frame(uint32_t timestamp_ms, const FragmentedFrame &frame) override;

  void to_csv(std::ostream & out);
};

#endif
