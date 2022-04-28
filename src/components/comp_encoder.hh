#ifndef COMP_ENCODER_HH
#define COMP_ENCODER_HH

#include "packet.hh"
#include "frame_observer.hh"
#include "optional.hh"

class EncoderInterface 
{
protected:
  std::vector<FrameObserverPtr> frame_observers_ {};

public:
  // main interface
  virtual void set_target_bitrate(uint32_t bitrate_byteps) = 0;
  virtual Optional<FragmentedFrame> encode_next_frame(uint32_t curr_timestamp_ms) = 0;

  // adding observer
  void add_frame_observer(FrameObserverPtr obs) { frame_observers_.push_back(obs); }

  virtual ~EncoderInterface() = default;
};

class BasicEncoder
  : public EncoderInterface
{
private:
  uint32_t target_bitrate_byteps_ {0};
  uint16_t fps_ {0};
  uint32_t frame_id_ {0};
  uint32_t gop_ {DEFAULT_GOP};
  uint8_t fec_rate_ {0};

  constexpr static int DEFAULT_GOP = 250; 

public:
  BasicEncoder(uint32_t init_bitrate, uint16_t fps);

  void set_gop(uint32_t gop) { gop_ = gop; }
  uint32_t gop() { return gop_; }

  virtual void set_target_bitrate(uint32_t bitrate_byteps) override {target_bitrate_byteps_ = bitrate_byteps;}
  virtual Optional<FragmentedFrame> encode_next_frame(uint32_t curr_timestamp_ms) override;

  void set_fec_rate(uint8_t fec_rate) { fec_rate_ = fec_rate; }
  uint8_t fec_rate() const { return fec_rate_; }
};

#endif
