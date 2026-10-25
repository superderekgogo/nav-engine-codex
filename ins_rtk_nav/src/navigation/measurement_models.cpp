#include "navcore/measurement_models.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace navcore {
namespace {
using SatelliteMap = std::map<int, const PreparedSatellite*>;

SatelliteMap indexEpoch(const PreparedGnssEpoch& epoch) {
  SatelliteMap result;
  for (const auto& satellite : epoch.satellites) {
    if (!result.emplace(satellite.svid, &satellite).second)
      throw std::runtime_error("duplicate SVID in prepared GNSS epoch");
  }
  return result;
}

double elevationScore(const PreparedSatellite& previous, const PreparedSatellite& current) {
  if (previous.satellite_elevation_deg && current.satellite_elevation_deg)
    return *previous.satellite_elevation_deg + *current.satellite_elevation_deg;
  if (current.satellite_elevation_deg) return 2.0 * *current.satellite_elevation_deg;
  if (previous.satellite_elevation_deg) return 2.0 * *previous.satellite_elevation_deg;
  return 0.0;
}

std::optional<double> carrier(const PreparedSatellite& satellite, CarrierPhaseBand band) {
  return band == CarrierPhaseBand::L1 ? satellite.rover_l1_cycles : satellite.rover_l2_cycles;
}
std::optional<double> baseCarrier(const PreparedSatellite& satellite, CarrierPhaseBand band) {
  return band == CarrierPhaseBand::L1 ? satellite.base_l1_cycles : satellite.base_l2_cycles;
}
std::optional<double> singleDifferenceDoppler(const PreparedSatellite& satellite) {
  if (!satellite.rover_d1_hz || !satellite.base_d1_hz) return std::nullopt;
  return *satellite.rover_d1_hz - *satellite.base_d1_hz;
}

bool phaseSlip(const PreparedSatellite& satellite, CarrierPhaseBand band) {
  if (band == CarrierPhaseBand::L1)
    return satellite.rover_l1_cycle_slip || satellite.base_l1_cycle_slip;
  return satellite.rover_l2_cycle_slip || satellite.base_l2_cycle_slip;
}

int initialPivot(const PreparedGnssEpoch& epoch, CarrierPhaseBand band) {
  int pivot = 0;
  double bestScore = -1.0;
  for (const auto& satellite : epoch.satellites) {
    if (!carrier(satellite, band) || !baseCarrier(satellite, band) || phaseSlip(satellite, band)) continue;
    const double score = satellite.satellite_elevation_deg.value_or(0.0);
    if (pivot == 0 || score > bestScore) { pivot = satellite.svid; bestScore = score; }
  }
  return pivot;
}

struct BandBuild {
  std::vector<TdDdCarrierPhaseObservation> observations;
  int pivot = 0;
  std::uint64_t slip_rejected = 0;
};

BandBuild buildBand(const SatelliteMap& previous, const SatelliteMap& current,
                    CarrierPhaseBand band, double wavelength, int fixedPivot,
                    std::size_t minimumCommonSatellites, double dt,
                    double dopplerPhaseRateSign, double dopplerToPhaseScale,
                    double slipThresholdCycles) {
  std::vector<int> common;
  std::uint64_t slipRejected = 0;
  for (const auto& item : current) {
    const auto old = previous.find(item.first);
    if (old == previous.end()) continue;
    const auto oldR = carrier(*old->second, band), oldB = baseCarrier(*old->second, band);
    const auto newR = carrier(*item.second, band), newB = baseCarrier(*item.second, band);
    const bool slipped = phaseSlip(*old->second, band) || phaseSlip(*item.second, band);
    if (slipped) { ++slipRejected; continue; }
    if (oldR && oldB && newR && newB) common.push_back(item.first);
  }
  if (common.size() < minimumCommonSatellites) return {{}, 0, slipRejected};

  int pivot = fixedPivot;
  if (pivot == 0 || !std::binary_search(common.begin(), common.end(), pivot)) {
    pivot = common.front();
    double bestScore = -1.0;
    for (const int svid : common) {
      const double score = elevationScore(*previous.at(svid), *current.at(svid));
      if (score > bestScore) { bestScore = score; pivot = svid; }
    }
    return {{}, pivot, slipRejected}; // New pivot: establish it as the reference before differencing.
  }

  BandBuild result; result.pivot = pivot; result.slip_rejected = slipRejected;
  const auto& oldPivot = *previous.at(pivot);
  const auto& newPivot = *current.at(pivot);
  const double oldPivotSd = *carrier(oldPivot, band) - *baseCarrier(oldPivot, band);
  const double newPivotSd = *carrier(newPivot, band) - *baseCarrier(newPivot, band);
  const auto oldPivotDoppler = singleDifferenceDoppler(oldPivot);
  const auto newPivotDoppler = singleDifferenceDoppler(newPivot);
  for (const int svid : common) {
    if (svid == pivot) continue;
    const auto& oldSat = *previous.at(svid);
    const auto& newSat = *current.at(svid);
    const double oldDd = (*carrier(oldSat, band) - *baseCarrier(oldSat, band)) - oldPivotSd;
    const double newDd = (*carrier(newSat, band) - *baseCarrier(newSat, band)) - newPivotSd;
    TdDdCarrierPhaseObservation observation;
    observation.svid = svid;
    observation.pivot_svid = pivot;
    observation.band = band;
    observation.wavelength_m = wavelength;
    observation.delta_double_difference_m = wavelength * (newDd - oldDd);
    const auto oldDoppler = singleDifferenceDoppler(oldSat);
    const auto newDoppler = singleDifferenceDoppler(newSat);
    if (oldDoppler && newDoppler && oldPivotDoppler && newPivotDoppler) {
      const double expectedCycles = dopplerPhaseRateSign * dopplerToPhaseScale * dt *
          (0.5 * (*oldDoppler + *newDoppler) - 0.5 * (*oldPivotDoppler + *newPivotDoppler));
      observation.doppler_phase_residual_cycles = (newDd - oldDd) - expectedCycles;
      observation.cycle_slip_screened = true;
      observation.cycle_slip_detected = std::abs(observation.doppler_phase_residual_cycles) > slipThresholdCycles;
    }
    if (observation.cycle_slip_detected) { ++result.slip_rejected; continue; }
    observation.satellite_position_previous_ecef_m = oldSat.satellite_position_ecef_m;
    observation.satellite_position_current_ecef_m = newSat.satellite_position_ecef_m;
    observation.pivot_position_previous_ecef_m = oldPivot.satellite_position_ecef_m;
    observation.pivot_position_current_ecef_m = newPivot.satellite_position_ecef_m;
    observation.elevation_previous_deg = oldSat.satellite_elevation_deg;
    observation.elevation_current_deg = newSat.satellite_elevation_deg;
    result.observations.push_back(std::move(observation));
  }

  return result;
}
}

TdDdCarrierPhaseBuilder::TdDdCarrierPhaseBuilder(TdDdBuilderConfig config) : config_(config) {
  if (!(config_.max_epoch_gap_seconds > 0.0) || !std::isfinite(config_.max_epoch_gap_seconds) ||
      !(config_.l1_wavelength_m > 0.0) || !(config_.l2_wavelength_m > 0.0) ||
      !(config_.phase_sigma_cycles > 0.0) || !std::isfinite(config_.phase_sigma_cycles) ||
      std::abs(std::abs(config_.doppler_phase_rate_sign) - 1.0) > 1e-12 ||
      !(config_.cycle_slip_threshold_cycles > 0.0) || !std::isfinite(config_.cycle_slip_threshold_cycles) ||
      config_.minimum_common_satellites < 2)
    throw std::runtime_error("invalid TD-DDCP builder configuration");
}

std::optional<PreparedTdDdCarrierPhaseEpoch> TdDdCarrierPhaseBuilder::push(const PreparedGnssEpoch& epoch) {
  ++summary_.input_epochs;
  if (!std::isfinite(epoch.gps_seconds)) throw std::runtime_error("non-finite GNSS epoch time in TD-DDCP builder");
  if (!previous_) {
    previous_ = epoch;
    pivotL1_ = initialPivot(epoch, CarrierPhaseBand::L1);
    pivotL2_ = initialPivot(epoch, CarrierPhaseBand::L2);
    ++summary_.priming_epochs;
    return std::nullopt;
  }

  const double dt = epoch.gps_seconds - previous_->gps_seconds;
  if (!(dt > 0.0)) throw std::runtime_error("GNSS epochs are not strictly increasing in TD-DDCP builder");
  if (dt > config_.max_epoch_gap_seconds) {
    ++summary_.rejected_gap_pairs;
    previous_ = epoch;
    ++summary_.priming_epochs;
    return std::nullopt;
  }

  const auto previousSatellites = indexEpoch(*previous_);
  const auto currentSatellites = indexEpoch(epoch);
  const int oldPivotL1 = pivotL1_, oldPivotL2 = pivotL2_;
  auto l1 = buildBand(previousSatellites, currentSatellites, CarrierPhaseBand::L1,
                      config_.l1_wavelength_m, pivotL1_, config_.minimum_common_satellites,
                      dt, config_.doppler_phase_rate_sign, 1.0, config_.cycle_slip_threshold_cycles);
  auto l2 = buildBand(previousSatellites, currentSatellites, CarrierPhaseBand::L2,
                      config_.l2_wavelength_m, pivotL2_, config_.minimum_common_satellites,
                      dt, config_.doppler_phase_rate_sign,
                      config_.l1_wavelength_m / config_.l2_wavelength_m,
                      config_.cycle_slip_threshold_cycles);
  pivotL1_ = l1.pivot;
  pivotL2_ = l2.pivot;
  summary_.cycle_slip_rejected_observations += l1.slip_rejected + l2.slip_rejected;

  const bool changed = (oldPivotL1 != 0 && pivotL1_ != oldPivotL1) || (oldPivotL2 != 0 && pivotL2_ != oldPivotL2);
  if (changed) {
    ++summary_.pivot_reset_pairs;
    previous_ = epoch;
    ++summary_.priming_epochs;
    return std::nullopt;
  }

  PreparedTdDdCarrierPhaseEpoch result;
  result.previous_gps_seconds = previous_->gps_seconds;
  result.gps_seconds = epoch.gps_seconds;
  result.sequence = sequence_++;
  result.pivot_l1_svid = pivotL1_;
  result.pivot_l2_svid = pivotL2_;
  result.reference_epoch_valid = true;
  result.observations.reserve(l1.observations.size() + l2.observations.size());
  result.observations.insert(result.observations.end(), l1.observations.begin(), l1.observations.end());
  result.observations.insert(result.observations.end(), l2.observations.begin(), l2.observations.end());

  // Build block-diagonal L1/L2 covariance blocks; each band includes shared-pivot correlation.
  const Eigen::Index n = static_cast<Eigen::Index>(result.observations.size());
  result.covariance_m2 = Eigen::MatrixXd::Zero(n, n);
  auto fillBand = [&](std::size_t offset, std::size_t count, double wavelength) {
    const double q = 4.0 * config_.phase_sigma_cycles * config_.phase_sigma_cycles * wavelength * wavelength;
    for (std::size_t i = 0; i < count; ++i)
      for (std::size_t j = 0; j < count; ++j)
        result.covariance_m2(static_cast<Eigen::Index>(offset + i), static_cast<Eigen::Index>(offset + j)) = q * ((i == j ? 1.0 : 0.0) + 1.0);
  };
  fillBand(0, l1.observations.size(), config_.l1_wavelength_m);
  fillBand(l1.observations.size(), l2.observations.size(), config_.l2_wavelength_m);
  result.status = result.observations.empty() ? "no_clean_phase_rows" : "ready_clean_rows";

  previous_ = epoch;
  if (result.observations.empty()) {
    ++summary_.epochs_without_valid_phase;
    return std::nullopt;
  }
  ++summary_.output_epochs;
  summary_.output_observations += result.observations.size();
  for (const auto& observation : result.observations) {
    if (observation.cycle_slip_screened) ++summary_.cycle_slip_screened_observations;
    if (observation.cycle_slip_detected) ++summary_.cycle_slip_rejected_observations;
  }
  return result;
}

void TdDdCarrierPhaseBuilder::reset() noexcept {
  previous_.reset();
  pivotL1_ = 0;
  pivotL2_ = 0;
  sequence_ = 0;
  summary_ = {};
}

} // namespace navcore
