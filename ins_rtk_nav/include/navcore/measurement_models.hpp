#pragma once

#include "navcore/messages.hpp"
#include <optional>

namespace navcore {

struct TdDdBuilderConfig {
  double max_epoch_gap_seconds = 1.5;
  double l1_wavelength_m = 0.190293672798;
  double l2_wavelength_m = 0.244210213425;
  double phase_sigma_cycles = 0.01; // per receiver, per satellite; initial model
  double doppler_phase_rate_sign = -1.0; // phase cycles/s = sign * Doppler Hz
  double cycle_slip_threshold_cycles = 0.5;
  std::size_t minimum_common_satellites = 2; // includes pivot
};

class TdDdCarrierPhaseBuilder {
public:
  explicit TdDdCarrierPhaseBuilder(TdDdBuilderConfig config);
  std::optional<PreparedTdDdCarrierPhaseEpoch> push(const PreparedGnssEpoch& epoch);
  const TdDdBuildSummary& summary() const noexcept { return summary_; }
  void reset() noexcept;

private:
  TdDdBuilderConfig config_;
  std::optional<PreparedGnssEpoch> previous_;
  int pivotL1_ = 0;
  int pivotL2_ = 0;
  std::uint64_t sequence_ = 0;
  TdDdBuildSummary summary_;
};

} // namespace navcore
