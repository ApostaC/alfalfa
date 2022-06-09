#ifndef COMP_ENCODER_HH
#define COMP_ENCODER_HH

#include "packet.hh"
#include "frame_observer.hh"
#include "optional.hh"
#include "stats.hh"

/**
 * basic interface for a encoder
 */
class EncoderInterface 
{
protected:
  std::vector<FrameObserverPtr> frame_observers_ {};

public:
  // main interface
  virtual void set_target_bitrate(uint32_t bitrate_byteps) = 0;
  virtual void set_loss_rate(double loss_rate) = 0;
  virtual Optional<FragmentedFrame> encode_next_frame(uint32_t curr_timestamp_ms) = 0;
  virtual std::vector<Packet> encode_next_frame_packets(uint32_t curr_timestamp_ms) = 0;

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

  double protection_overhead_ {1};

  constexpr static int DEFAULT_GOP = 250; 

  StatsRecorder stats_recoder_;

public:
  BasicEncoder(uint32_t init_bitrate, uint16_t fps);
  BasicEncoder(uint32_t init_bitrate, uint16_t fps, const std::string &stats_file);

  void set_gop(uint32_t gop) { gop_ = gop; }
  uint32_t gop() const { return gop_; }

  virtual void set_target_bitrate(uint32_t bitrate_byteps) override {target_bitrate_byteps_ = bitrate_byteps;}
  virtual void set_loss_rate(double loss_rate) override;
  virtual Optional<FragmentedFrame> encode_next_frame(uint32_t curr_timestamp_ms) override;
  virtual std::vector<Packet> encode_next_frame_packets(uint32_t curr_timestamp_ms) override;

  /**
   * set the protection overhead: [1.0, 2.0]
   * fec_ratio = max(protection_overhead * estimated_loss_rate, 50)
   * fec_rate = fec_ratio / (1 - fec_ratio)
   */
  void set_protection_overhead(double overhead) { protection_overhead_ = overhead; };
  void set_fec_rate(uint8_t fec_rate) { fec_rate_ = fec_rate; }
  uint8_t fec_rate() const { return fec_rate_; }

  ~BasicEncoder() { stats_recoder_.dump(true); }
};

class SVCEncoder : public EncoderInterface
{
private:
  constexpr static int DEFAULT_GOP = 250; 

  std::vector<BasicEncoder> layer_encoders_ {};
  std::vector<double> size_ratios_ {};
  std::vector<uint8_t> fec_rates_ {};
  uint32_t frame_id_ {0};
  uint32_t gop_{DEFAULT_GOP};

  StatsRecorder stats_recoder_;

public:
  /**
   * order: from base layer to highest layer
   */
  SVCEncoder(uint32_t init_bitrate, uint16_t fps,
             const std::vector<double> & size_ratios, const std::vector<uint8_t> & fec_rates);

  SVCEncoder(uint32_t init_bitrate, uint16_t fps,
             const std::vector<double> & size_ratios, const std::vector<uint8_t> & fec_rates,
             const std::string & stats_file);

  virtual void set_target_bitrate(uint32_t bitrate_byteps) override; 
  virtual void set_loss_rate(double) override {} /* do nothing when loss, because we add a fixed fec rate */
  virtual Optional<FragmentedFrame> encode_next_frame(uint32_t curr_timestamp_ms) override;
  virtual std::vector<Packet> encode_next_frame_packets(uint32_t curr_timestamp_ms) override;

  /* other setter/getters */
  void set_gop(uint32_t gop);
  uint32_t gop() const { return layer_encoders_.front().gop(); }
};

#endif
