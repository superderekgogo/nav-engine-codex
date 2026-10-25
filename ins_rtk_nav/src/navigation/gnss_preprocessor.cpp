#include "navcore/gnss_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace navcore {
namespace {

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2) return *middle;
  const double upper = *middle;
  const auto lower = std::max_element(values.begin(), middle);
  return 0.5 * (*lower + upper);
}

struct PendingCheck {
  PreparedSatellite* satellite = nullptr;
  bool rover = false;
  std::optional<double> l1_residual_cycles;
  std::optional<double> geometry_free_delta_m;
  bool l1_reacquired = false;
  bool l2_reacquired = false;
};

} // namespace

GnssCycleSlipPreprocessor::GnssCycleSlipPreprocessor(GnssCycleSlipConfig config)
    : config_(config) {
  if (!(config_.maximum_epoch_gap_seconds > 0.0) ||
      !(config_.doppler_phase_threshold_cycles > 0.0) ||
      !(config_.integer_cycle_tolerance_cycles > 0.0 && config_.integer_cycle_tolerance_cycles < 0.5) ||
      !(config_.geometry_free_threshold_m > 0.0) ||
      !(config_.l1_wavelength_m > 0.0) || !(config_.l2_wavelength_m > 0.0) ||
      config_.minimum_common_satellites < 2)
    throw std::invalid_argument("invalid GNSS cycle-slip preprocessing configuration");
}

PreparedGnssEpoch GnssCycleSlipPreprocessor::process(const PreparedGnssEpoch& input) {
  PreparedGnssEpoch output = input;
  ++summary_.input_epochs;
  std::vector<PendingCheck> pending;
  std::vector<double> roverResiduals, baseResiduals;

  for (auto& satellite : output.satellites) {
    for (int receiver = 0; receiver < 2; ++receiver) {
      const bool rover = receiver == 0;
      auto& historyMap = rover ? roverHistory_ : baseHistory_;
      auto& history = historyMap[satellite.svid];
      const auto& l1 = rover ? satellite.rover_l1_cycles : satellite.base_l1_cycles;
      const auto& l2 = rover ? satellite.rover_l2_cycles : satellite.base_l2_cycles;
      const auto& d1 = rover ? satellite.rover_d1_hz : satellite.base_d1_hz;
      PendingCheck check;
      check.satellite = &satellite;
      check.rover = rover;

      const double dt = input.gps_seconds - history.time;
      const bool continuous = history.time > 0.0 && dt > 0.0 && dt <= config_.maximum_epoch_gap_seconds;
      check.l1_reacquired = history.time > 0.0 && l1 &&
                            ((!history.has_l1) || (history.has_l1 && dt > config_.maximum_epoch_gap_seconds));
      check.l2_reacquired = history.time > 0.0 && l2 &&
                            ((!history.has_l2) || (history.has_l2 && dt > config_.maximum_epoch_gap_seconds));
      if (continuous && l1 && d1 && history.has_l1 && history.has_d1) {
        // RTKLIB uses phase_rate = -Doppler. The receiver clock jump is
        // removed below as a robust common-mode median across satellites.
        check.l1_residual_cycles = (*l1 - history.l1_cycles) +
                                   0.5 * (*d1 + history.d1_hz) * dt;
        (rover ? roverResiduals : baseResiduals).push_back(*check.l1_residual_cycles);
      }
      if (continuous && l1 && l2 && history.has_l1 && history.has_l2) {
        // Geometry and receiver clock cancel in the dual-frequency
        // geometry-free phase; the remaining smooth term is ionospheric.
        check.geometry_free_delta_m = config_.l1_wavelength_m * (*l1 - history.l1_cycles) -
                                      config_.l2_wavelength_m * (*l2 - history.l2_cycles);
      }
      pending.push_back(check);

      history.time = input.gps_seconds;
      history.has_l1 = l1.has_value();
      history.has_l2 = l2.has_value();
      history.has_d1 = d1.has_value();
      if (l1) history.l1_cycles = *l1;
      if (l2) history.l2_cycles = *l2;
      if (d1) history.d1_hz = *d1;
    }
  }

  const double roverCommon = roverResiduals.size() >= config_.minimum_common_satellites ? median(roverResiduals) : 0.0;
  const double baseCommon = baseResiduals.size() >= config_.minimum_common_satellites ? median(baseResiduals) : 0.0;
  for (const auto& check : pending) {
    auto& satellite = *check.satellite;
    const double common = check.rover ? roverCommon : baseCommon;
    bool l1Slip = check.l1_reacquired;
    bool l2Slip = check.l2_reacquired;
    if (check.l1_residual_cycles) {
      const double residual = *check.l1_residual_cycles - common;
      if (std::abs(residual) > config_.doppler_phase_threshold_cycles) {
        const long long l1SlipCycles = std::llround(residual);
        const bool integerLike = std::abs(residual - static_cast<double>(l1SlipCycles)) <=
                                 config_.integer_cycle_tolerance_cycles;
        if (integerLike && check.geometry_free_delta_m) {
          // Doppler/phase alone produced many non-integer outliers in this
          // dataset. Confirm an L1 jump against the independent geometry-free
          // carrier combination before resetting the RTK ambiguity state.
          const double afterL1 = *check.geometry_free_delta_m -
                                 config_.l1_wavelength_m * static_cast<double>(l1SlipCycles);
          if (std::abs(afterL1) <= config_.geometry_free_threshold_m) {
            l1Slip = true;
            const long long l2SlipCycles = std::llround(-afterL1 / config_.l2_wavelength_m);
            l2Slip = l2SlipCycles != 0 &&
                     std::abs(afterL1 + config_.l2_wavelength_m * static_cast<double>(l2SlipCycles)) <=
                         config_.geometry_free_threshold_m;
          } else {
            ++summary_.rejected_l1_candidates;
          }
        } else {
          ++summary_.rejected_l1_candidates;
        }
      }
    }

    if (check.geometry_free_delta_m && !l1Slip && !l2Slip) {
      // With no L1 corroboration, identify the more likely single-band integer
      // jump from the dual-frequency phase change. The configured tolerance is
      // below the L1/L2 wavelength separation to avoid classifying one band as
      // the other in the common single-slip case.
      const double gf = *check.geometry_free_delta_m;
      const long long n1 = std::llround(gf / config_.l1_wavelength_m);
      const long long n2 = std::llround(-gf / config_.l2_wavelength_m);
      const double e1 = std::abs(gf - config_.l1_wavelength_m * static_cast<double>(n1));
      const double e2 = std::abs(gf + config_.l2_wavelength_m * static_cast<double>(n2));
      if (n1 != 0 && e1 <= config_.geometry_free_threshold_m && e1 < e2) l1Slip = true;
      else if (n2 != 0 && e2 <= config_.geometry_free_threshold_m) l2Slip = true;
    }

    if (l1Slip) {
      if (check.rover) { satellite.rover_l1_cycle_slip = true; ++summary_.flagged_rover_l1; }
      else { satellite.base_l1_cycle_slip = true; ++summary_.flagged_base_l1; }
      ++summary_.flagged_l1;
    }
    if (l2Slip) {
      if (check.rover) { satellite.rover_l2_cycle_slip = true; ++summary_.flagged_rover_l2; }
      else { satellite.base_l2_cycle_slip = true; ++summary_.flagged_base_l2; }
      ++summary_.flagged_l2;
    }
    // Publish cleaned epochs: downstream modules cannot accidentally consume
    // a carrier phase that this preprocessor rejected. Keep the flags for LLI
    // propagation and diagnostics, and retain code/Doppler observations.
    if (satellite.rover_l1_cycle_slip) satellite.rover_l1_cycles.reset();
    if (satellite.base_l1_cycle_slip) satellite.base_l1_cycles.reset();
    if (satellite.rover_l2_cycle_slip) satellite.rover_l2_cycles.reset();
    if (satellite.base_l2_cycle_slip) satellite.base_l2_cycles.reset();
  }
  return output;
}

} // namespace navcore
