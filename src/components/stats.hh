#ifndef STATS_HH
#define STATS_HH

#include <vector>

#include "congestion_control.hh"

/**
 * Record the stats timeseries, includes pacing rate, target rate and packet losses
 */
class StatsRecorder : public CongestionControlObserver
{
private:
  struct Elem 
  {
    double timestamp_ms;
    uint32_t pacing_rate_byteps;
    uint32_t target_rate_byteps;
    double loss_rate;
  };

private:
  uint32_t start_timestamp_ {0};
  std::vector<Elem> records_ {};
  const std::string outfile_ {};

public:
  StatsRecorder(const std::string &outfile);

  virtual void post_updates(uint32_t sending_rate_byteps, uint32_t target_rate_byteps, double loss) override;

  void dump(bool relative_time = true);
};

#endif
