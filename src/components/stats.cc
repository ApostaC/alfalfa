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
