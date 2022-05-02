#include <vector>
#include <iostream>
#include <random>
#include "comp_encoder.hh"
#include "timestamp.hh"

using namespace std;

BasicEncoder::BasicEncoder(uint32_t init_bitrate_byteps, uint16_t fps)
  : target_bitrate_byteps_(init_bitrate_byteps), fps_(fps)
{
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
  cerr << "Encoding a new frame: " << frame_id_ << ", size = " << real_bytes << " tgt_br = " << target_bitrate_byteps_ << endl;
  return ret;
}
