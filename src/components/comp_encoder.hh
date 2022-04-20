#ifndef COMP_ENCODER_HH
#define COMP_ENCODER_HH

#include "packet.hh"

class EncoderInterface 
{
public:
  virtual void set_target_bitrate(uint32_t bitrate_byteps) = 0;
  virtual FragmentedFrame encode_next_frame(uint32_t curr_timestamp_ms) = 0;

  virtual ~EncoderInterface() = default;
};

class BasicEncoder
  : public EncoderInterface
{
private:
  uint32_t target_bitrate_byteps_ {0};
  uint16_t fps_ {0};
  uint32_t frame_id_ {0};

public:
  BasicEncoder(uint32_t init_bitrate, uint16_t fps);

  virtual void set_target_bitrate(uint32_t bitrate_byteps) override {target_bitrate_byteps_ = bitrate_byteps;}
  virtual FragmentedFrame encode_next_frame(uint32_t curr_timestamp_ms) override;
};

#endif
