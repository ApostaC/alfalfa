#include <vector>
#include <cassert>
#include <iostream>
#include <random>
#include "comp_encoder.hh"
#include "timestamp.hh"

using namespace std;

BasicEncoder::BasicEncoder(uint32_t init_bitrate_byteps, uint16_t fps)
  : target_bitrate_byteps_(init_bitrate_byteps), fps_(fps)
{
}

void BasicEncoder::set_loss_rate(double loss_rate)
{
  // compute FEC rate
  auto fec_ratio = min(loss_rate * protection_overhead_, 0.5);
  auto fec_rate_d = min(fec_ratio / (1 - fec_ratio), 1.);
  fec_rate_ = 255 * fec_rate_d;
}

Optional<FragmentedFrame> BasicEncoder::encode_next_frame(uint32_t curr_timestamp_ms)
{
  frame_id_ += 1;
  std::random_device rd;
  std::ranlux24 e(rd());
  std::normal_distribution<double> dis(1, 1e-5);
  auto multiplier = dis(e);
  uint32_t base_size_bytes = target_bitrate_byteps_ * 1 / fps_;

  /* consider fec */
  base_size_bytes = base_size_bytes / (1 + (fec_rate_ / 255.0f));

  uint32_t real_bytes = std::floor(multiplier * base_size_bytes);
  if (real_bytes == 0) {
    //return {};
    real_bytes = 10; // generate a very small frame
  }
  std::vector<uint8_t> data;
  data.resize(real_bytes);

  // note: time_to_next_frame is in MICRO-SECONDS (us)
  bool is_key_frame = (frame_id_ % gop_ == 1);
  FragmentedFrame ret(0, 0, 0, frame_id_, 1000000 / fps_, data, is_key_frame, fec_rate_);

  // update frame_observer
  for (auto obs : frame_observers_) {
    obs->new_complete_frame(curr_timestamp_ms, ret);
  }
  cerr << "Encoding a new frame: " << frame_id_ << ", size = " << real_bytes 
       << " tgt_br = " << target_bitrate_byteps_ << ", fec_rate = " << fec_rate_ / 255.0 << endl;
  return ret;
}

std::vector<Packet> BasicEncoder::encode_next_frame_packets(uint32_t curr_timestamp_ms)
{
  auto frame = encode_next_frame(curr_timestamp_ms);
  if (frame.initialized()) {
    return frame.get().packets();
  }
  return {};
}

/* SVC Codec */
SVCEncoder::SVCEncoder(uint32_t init_bitrate, uint16_t fps,
                       const vector<double> & size_ratios, const vector<uint8_t> & fec_rates)
  : size_ratios_(size_ratios), fec_rates_(fec_rates)
{
  /* initialize layer_encoders_ */
  assert(size_ratios.size() == fec_rates.size());
  for (unsigned i = 0; i < size_ratios.size(); i++) {
    uint32_t init_rate = init_bitrate * size_ratios[i];
    auto fec_rate = fec_rates[i];
    layer_encoders_.emplace_back(init_rate, fps);

    /* set fec rate */
    layer_encoders_.back().set_fec_rate(fec_rate);
    layer_encoders_.back().set_protection_overhead(1);
  }
}

void SVCEncoder::set_target_bitrate(uint32_t bitrate_byteps) 
{
  for (unsigned i = 0; i < size_ratios_.size(); i++) {
    uint32_t rate_byteps = bitrate_byteps * size_ratios_[i];
    layer_encoders_[i].set_target_bitrate(rate_byteps);
  }
}

void SVCEncoder::set_gop(uint32_t gop)
{
  gop_ = gop;
  for (auto & encoder : layer_encoders_) {
    encoder.set_gop(gop);
  }
}

Optional<FragmentedFrame> SVCEncoder::encode_next_frame(uint32_t)
{
  throw runtime_error("Please call SVCEncoder::encode_next_frame_packets instead of SVCEncoder::encode_next_frame");
}

std::vector<Packet> SVCEncoder::encode_next_frame_packets(uint32_t curr_timestamp_ms)
{
  frame_id_ += 1;
  /* encode layers */
  std::vector<FragmentedFrame> temp_frames;
  uint32_t layer_id = 0;
  for (auto & encoder : layer_encoders_) {
    auto frame_handle = encoder.encode_next_frame(curr_timestamp_ms);
    if (not frame_handle.initialized()) {
      throw runtime_error("SVCEncoder::encode_next_frame_packets: failed to encode a new frame!");
    }

    temp_frames.emplace_back(move(frame_handle.get()));
    temp_frames.back().set_frame_no(layer_id);
    layer_id++;
  }

  /* combine to get a new svc frame */
  bool is_key_frame = (frame_id_ % gop_ == 1);
  SVCFrame svc_frame(frame_id_, move(temp_frames));
  if (is_key_frame) svc_frame.set_key_frame();

  for (auto obs : frame_observers_) {
    obs->new_complete_frame(curr_timestamp_ms, svc_frame.to_fragmented_frame().get());
  }
  return svc_frame.fragments();
}
