#include <iostream>
#include <fstream>
#include "timestamp.hh"
#include "stats.hh"

using namespace std;

StatsRecorder::StatsRecorder(const std::string & outfile)
  : outfile_(outfile)
{}

void StatsRecorder::post_updates(uint32_t sending_rate_byteps, uint32_t target_rate_byteps, double loss)
{
  double now_ms = timestamp_us() / 1000.0;
  if (start_timestamp_ == 0) {
    start_timestamp_ = now_ms;
  }

  /* add the prev record with the current timestamp for plotting */
  if (records_.size() > 0) {
    Elem prev_rec = records_.back();
    prev_rec.timestamp_ms = now_ms;
    records_.push_back(prev_rec);
  }

  /* add the new record */
  Elem rec{now_ms, sending_rate_byteps, target_rate_byteps, loss};
  records_.push_back(rec);
}

void StatsRecorder::dump(bool relative_time)
{
  uint32_t now_ms = timestamp_ms();
  if (records_.size() > 0) {
    Elem prev_rec = records_.back();
    prev_rec.timestamp_ms = now_ms;
    records_.push_back(prev_rec);
  }

  ofstream fout(outfile_);
  if (!fout) {
    throw runtime_error("StatsRecorder::dump: cannot open file to write, file name is " + outfile_);
  }
  
  int32_t time_offset = relative_time ? start_timestamp_ : 0;
  for (auto rec : records_) {
    fout << rec.timestamp_ms - time_offset << "," << rec.pacing_rate_byteps
         << "," << rec.target_rate_byteps << "," << rec.loss_rate << endl;
  }
}

DecoderStats::DecoderStats(const string & outfile)
  : outfile_(outfile)
{
}

void DecoderStats::on_packet_received(uint32_t, const Packet & pkt)
{
  /**
   * when a packet arrives, there are 4 possible stats:
   *  found in normal, found in rtx: INVALID
   *  found in normal, not found in rtx: SKIP
   *  not found in normal, found in rtx: SKIP 
   *  not foudn in normal, not found in rtx: insert
   */
  auto frame_id = pkt.frame_no();
  auto frag_id = pkt.fragment_no();
  auto in_normal = normal_pkts_[frame_id].count(frag_id);
  auto in_rtx = rtx_pkts_[frame_id].count(frag_id);

  if (in_normal and in_rtx) {
    cerr << "Warning: packet is found in both normal and rtx!" << endl;
  }

  if (not in_normal and not in_rtx) {
    if (pkt.is_retrans()) {
      rtx_pkts_[frame_id].insert(frag_id);
    }
    else {
      normal_pkts_[frame_id].insert(frag_id);
    }
  }
  
  if (fec_ratio_.count(frame_id) == 0) {
    double fec_ratio = pkt.fec_rate() / 255.0;
    double loss_tolerance = fec_ratio / (1 + fec_ratio);
    fec_ratio_[frame_id] = loss_tolerance;
  }
}

void DecoderStats::dump() const
{
  ofstream fout(outfile_);
  if (!fout) {
    throw runtime_error("DecoderStats::dump(): cannot open file to write, file name is " + outfile_);
  }

  cerr << "HERE: dumping decoder stats to " << outfile_ << endl;

  /* get all frame_ids */
  std::set<uint32_t> frame_ids;
  std::map<uint32_t, double> loss_rates;
  for (auto & ent : normal_pkts_) {
    frame_ids.insert(ent.first);
  }
  for (auto & ent : rtx_pkts_) {
    frame_ids.insert(ent.first);
  }

  for (uint32_t frame_id : frame_ids) {
    auto normal_cnt = 0;
    auto rtx_cnt = 0;
    if (normal_pkts_.count(frame_id)) {
      normal_cnt = normal_pkts_.at(frame_id).size();
    }
    if (rtx_pkts_.count(frame_id)) {
      rtx_cnt = rtx_pkts_.at(frame_id).size();
    }

    loss_rates[frame_id] = 1. * rtx_cnt / (rtx_cnt + normal_cnt);
  }

  for (auto frame_id : frame_ids) {
    fout << frame_id << "," << fec_ratio_.at(frame_id) << "," << loss_rates[frame_id] << endl;
  }
}
