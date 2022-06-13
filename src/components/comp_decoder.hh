#ifndef COMP_DECODER_HH
#define COMP_DECODER_HH

#include <vector>
#include "frame_observer.hh"
#include "stats.hh"

class DecoderInterface
{
protected:
  std::vector<FrameObserverPtr> frame_observers_ {};
  DecoderStats stats_;

public:
  DecoderInterface(const std::string & stats_file) : stats_(stats_file) {}
  // main interface
  virtual void incoming_packet(uint32_t timestamp_ms, const Packet &pkt) = 0;

  // adding observer
  void add_frame_observer(FrameObserverPtr obs) { frame_observers_.push_back(obs); }
  
  // get stats
  const DecoderStats & get_stats() const { return stats_; }

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
  NonBlockingDecoder();
  NonBlockingDecoder(const std::string & stats_file);

  virtual void incoming_packet(uint32_t timestamp_ms, const Packet &pkt) override;

};

/**
 * wait until the current frame has arrived or an I frame has arrived!
 */
class BlockingDecoder : public DecoderInterface
{
private:
  std::map<uint32_t, FragmentedFrame> frames_ {};
  uint32_t already_decoded_no_ {0};

private:
  void on_frame_complete(uint32_t timestamp_ms, const FragmentedFrame & frame);

public:
  BlockingDecoder();
  BlockingDecoder(const std::string & stats_file);

  virtual void incoming_packet(uint32_t timestamp_ms, const Packet & pkt);

};

class SVCDecoder : public DecoderInterface
{
private:
  std::map<uint32_t, SVCFrame> frames_ {};
  unsigned num_layers_ {};
  uint32_t already_decoded_no_ {0};
  uint32_t latest_frame_no_ {0};

private:
  void on_frame_complete(uint32_t timestamp_ms, const SVCFrame & frame);

public: 
  SVCDecoder(unsigned num_layers, const std::string & stats_file) 
    : DecoderInterface(stats_file), num_layers_(num_layers) {}
  SVCDecoder(unsigned num_layers) : DecoderInterface("/tmp/decoder.csv"), num_layers_(num_layers) {}

  virtual void incoming_packet(uint32_t timestamp_ms, const Packet & pkt);

};

#endif
