#ifndef COMP_DECODER_HH
#define COMP_DECODER_HH

#include <vector>
#include "frame_observer.hh"

class DecoderInterface
{
protected:
  std::vector<FrameObserverPtr> frame_observers_ {};

public:
  // main interface
  virtual void incoming_packet(uint32_t timestamp_ms, const Packet &pkt) = 0;

  // adding observer
  void add_frame_observer(FrameObserverPtr obs) { frame_observers_.push_back(obs); }
  
  virtual ~DecoderInterface() = default;
};


/**
 * If a new frame is arrived, push all the previous frames to the registered FrameObserver
 */
class NonBlockingDecoder: public DecoderInterface
{
private:
  std::map<uint32_t, FragmentedFrame> frames_ {};
  uint32_t expected_frame_no_ {1};

private:
  void flush_frame(uint32_t timestamp_ms, const FragmentedFrame &frame);

public:
  NonBlockingDecoder() = default;

  virtual void incoming_packet(uint32_t timestamp_ms, const Packet &pkt) override;
};

#endif
