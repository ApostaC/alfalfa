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
  (void)(curr_timestamp_ms); // suppress unused
  std::random_device rd;
  std::ranlux24 e(rd());
  std::normal_distribution<double> dis(1, 1e-5);
  auto multiplier = dis(e);
  uint32_t base_size_bytes = target_bitrate_byteps_ * 1 / fps_;
  uint32_t real_bytes = std::floor(multiplier * base_size_bytes);
  if (real_bytes == 0) {
    return {};
  }
  std::vector<uint8_t> data;
  data.resize(real_bytes);
  // note: time_to_next_frame is in MICRO-SECONDS (us)
  bool is_key_frame = (frame_id_ % gop_ == 1);
  return FragmentedFrame(0, 0, 0, frame_id_, 1000000 / fps_, data, is_key_frame);
}
