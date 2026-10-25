#pragma once

#include "navcore/messages.hpp"
#include <cstdint>
#include <map>

namespace navcore {

struct GnssCycleSlipConfig {
  double maximum_epoch_gap_seconds = 1.5;
  double doppler_phase_threshold_cycles = 0.5;
  double integer_cycle_tolerance_cycles = 0.25;
  double geometry_free_threshold_m = 0.05;
  double l1_wavelength_m = 0.190293672798;
  double l2_wavelength_m = 0.244210213425;
  std::size_t minimum_common_satellites = 4;
};

struct GnssCycleSlipSummary {
  std::uint64_t input_epochs = 0;
  std::uint64_t flagged_l1 = 0;
  std::uint64_t flagged_l2 = 0;
  std::uint64_t rejected_l1_candidates = 0;
  std::uint64_t flagged_rover_l1 = 0;
  std::uint64_t flagged_base_l1 = 0;
  std::uint64_t flagged_rover_l2 = 0;
  std::uint64_t flagged_base_l2 = 0;
};

class GnssCycleSlipPreprocessor {
public:
  explicit GnssCycleSlipPreprocessor(GnssCycleSlipConfig config);
  PreparedGnssEpoch process(const PreparedGnssEpoch& epoch);
  const GnssCycleSlipSummary& summary() const noexcept { return summary_; }

private:
  struct SignalHistory {
    double time = 0.0;
    double l1_cycles = 0.0;
    double l2_cycles = 0.0;
    double d1_hz = 0.0;
    bool has_l1 = false;
    bool has_l2 = false;
    bool has_d1 = false;
  };
  GnssCycleSlipConfig config_;
  std::map<int, SignalHistory> roverHistory_, baseHistory_;
  GnssCycleSlipSummary summary_;
};

} // namespace navcore
