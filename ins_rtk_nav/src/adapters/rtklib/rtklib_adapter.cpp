#include "navcore/rtklib_adapter.hpp"

extern "C" {
#include "rtklib.h"
}

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace navcore {
struct RtklibAdapter::Impl {
  explicit Impl(RtkAdapterConfig value) : config(value) {
    if (config.frequency_count < 1 || config.frequency_count > 2 ||
        !(config.ambiguity_ratio_threshold >= 1.0) ||
        config.minimum_lock_count < 0 || config.minimum_fix_count < 1)
      throw std::invalid_argument("invalid RTK ambiguity or cycle-slip configuration");
    options = prcopt_default;
    options.mode = PMODE_KINEMA;
    options.nf = config.frequency_count;
    options.navsys = config.nav_system;
    options.sateph = EPHOPT_PREC;
    options.dynamics = 1;
    options.ionoopt = IONOOPT_BRDC;
    options.tropopt = TROPOPT_SAAS;
    if (config.ambiguity_mode == "off") options.modear = ARMODE_OFF;
    else if (config.ambiguity_mode == "continuous") options.modear = ARMODE_CONT;
    else if (config.ambiguity_mode == "instantaneous") options.modear = ARMODE_INST;
    else if (config.ambiguity_mode == "fix_and_hold") options.modear = ARMODE_FIXHOLD;
    else if (config.ambiguity_mode == "wide_lane_narrow_lane") options.modear = ARMODE_WLNL;
    else throw std::invalid_argument("unsupported RTK ambiguity resolution mode: " + config.ambiguity_mode);
    options.thresar[0] = config.ambiguity_ratio_threshold;
    options.minlock = config.minimum_lock_count;
    options.minfix = config.minimum_fix_count;
    options.refpos = 0;
    options.elmin = 10.0 * D2R;
    options.maxtdiff = 1e-3;
    options.maxout = 5;
  }
  RtkAdapterConfig config;
  prcopt_t options{};
};

RtklibAdapter::RtklibAdapter(RtkAdapterConfig config) : impl_(new Impl(config)) {}
RtklibAdapter::~RtklibAdapter() { delete impl_; }

std::vector<RtkEpochResult> RtklibAdapter::solve(const std::vector<PreparedGnssEpoch>& epochs) {
  std::vector<RtkEpochResult> results(epochs.size());
  if (epochs.empty()) return results;

  // RTKLIB's precise-orbit interpolator needs at least 11 samples. The input
  // satellite coordinates are already ECEF at reception time, so they are
  // passed through its precise-ephemeris data structure and interpolated to
  // transmission time. A synthetic broadcast clock record is supplied because
  // RTKLIB's satposs() asks for that clock before calling peph2pos().
  nav_t nav{};
  std::vector<peph_t> precise(epochs.size());
  std::vector<eph_t> clocks;
  std::vector<std::vector<std::optional<Eigen::Vector4d>>> satelliteStates(MAXSAT, std::vector<std::optional<Eigen::Vector4d>>(epochs.size()));
  for (std::size_t i = 0; i < epochs.size(); ++i) {
    precise[i].time = gpst2time(0, epochs[i].gps_seconds);
    for (const auto& satellite : epochs[i].satellites) {
      if (satellite.svid <= 0 || satellite.svid > MAXPRNGPS || !satellite.satellite_clock_m) continue;
      const int sat = satno(SYS_GPS, satellite.svid);
      if (!sat) continue;
      Eigen::Vector4d state;
      for (int axis = 0; axis < 3; ++axis) state[axis] = satellite.satellite_position_ecef_m[axis];
      state[3] = *satellite.satellite_clock_m / CLIGHT;
      satelliteStates[sat - 1][i] = state;
      eph_t eph{};
      eph.sat = sat;
      eph.toe = precise[i].time;
      eph.toc = precise[i].time;
      eph.f0 = state[3];
      clocks.push_back(eph);
    }
  }
  for (int sat = 0; sat < MAXSAT; ++sat) {
    auto& states = satelliteStates[sat];
    std::vector<std::size_t> valid;
    for (std::size_t i = 0; i < states.size(); ++i) if (states[i]) valid.push_back(i);
    if (valid.size() < 2) continue;
    for (std::size_t i = 0; i < states.size(); ++i) {
      if (!states[i]) {
        auto right = std::lower_bound(valid.begin(), valid.end(), i);
        std::size_t leftIndex, rightIndex;
        if (right == valid.begin()) { leftIndex = valid[0]; rightIndex = valid[1]; }
        else if (right == valid.end()) { leftIndex = valid[valid.size() - 2]; rightIndex = valid.back(); }
        else { rightIndex = *right; leftIndex = *(right - 1); }
        const double t0 = epochs[leftIndex].gps_seconds, t1 = epochs[rightIndex].gps_seconds;
        const double alpha = (epochs[i].gps_seconds - t0) / (t1 - t0);
        states[i] = *states[leftIndex] + alpha * (*states[rightIndex] - *states[leftIndex]);
      }
      for (int axis = 0; axis < 4; ++axis) precise[i].pos[sat][axis] = (*states[i])[axis];
    }
  }
  nav.peph = precise.data(); nav.ne = static_cast<int>(precise.size()); nav.nemax = nav.ne;
  nav.eph = clocks.data(); nav.n = static_cast<int>(clocks.size()); nav.nmax = nav.n;
  for (int sat = 0; sat < NSATGPS; ++sat) {
    nav.lam[sat][0] = CLIGHT / FREQ1;
    nav.lam[sat][1] = CLIGHT / FREQ2;
  }

  prcopt_t options = impl_->options;
  for (int i = 0; i < 3; ++i) options.rb[i] = epochs.front().base_station_position_ecef_m[i];
  rtk_t rtk{};
  rtkinit(&rtk, &options);
  for (int axis = 0; axis < 3; ++axis) rtk.sol.rr[axis] = impl_->config.initial_rover_position_ecef_m[axis];

  for (std::size_t epochIndex = 0; epochIndex < epochs.size(); ++epochIndex) {
    const auto& epoch = epochs[epochIndex];
    std::vector<obsd_t> obs;
    const gtime_t time = gpst2time(0, epoch.gps_seconds);
    auto append = [&](const PreparedSatellite& s, int receiver, int sat) {
      if (!sat || sat > MAXSAT) return;
      const auto& c1 = receiver == 1 ? s.rover_c1_m : s.base_c1_m;
      if (!s.rover_c1_m || !s.base_c1_m || !c1) return;
      obsd_t record{}; record.time = time; record.sat = static_cast<unsigned char>(sat); record.rcv = static_cast<unsigned char>(receiver);
      record.P[0] = *c1;
      record.code[0] = CODE_L1C; record.SNR[0] = 180;
      const auto& p2 = receiver == 1 ? s.rover_p2_m : s.base_p2_m;
      if (impl_->config.use_l2_pseudorange && p2) { record.P[1] = *p2; record.code[1] = CODE_L2C; record.SNR[1] = 180; }
      const auto& l2 = receiver == 1 ? s.rover_l2_cycles : s.base_l2_cycles;
      const bool l1Slip = receiver == 1 ? s.rover_l1_cycle_slip : s.base_l1_cycle_slip;
      const bool l2Slip = receiver == 1 ? s.rover_l2_cycle_slip : s.base_l2_cycle_slip;
      if (l1Slip) record.LLI[0] |= 1;
      if (l2Slip) record.LLI[1] |= 1;
      const auto& l1 = receiver == 1 ? s.rover_l1_cycles : s.base_l1_cycles;
      if (l1 && !l1Slip) record.L[0] = *l1;
      if (l2 && !l2Slip) { record.L[1] = *l2; record.code[1] = CODE_L2C; record.SNR[1] = 180; }
      const auto& d1 = receiver == 1 ? s.rover_d1_hz : s.base_d1_hz;
      if (d1) record.D[0] = static_cast<float>(*d1);
      obs.push_back(record);
    };
    for (const auto& s : epoch.satellites) {
      const int sat = satno(SYS_GPS, s.svid);
      if (!sat) continue;
      append(s, 1, sat); append(s, 2, sat);
    }
    std::stable_sort(obs.begin(), obs.end(), [](const obsd_t& a, const obsd_t& b) {
      if (a.rcv != b.rcv) return a.rcv < b.rcv;
      return a.sat < b.sat;
    });
    int common = 0;
    for (const auto& s : epoch.satellites) {
      const bool commonL1 = s.rover_l1_cycles && s.base_l1_cycles;
      const bool commonL2 = s.rover_l2_cycles && s.base_l2_cycles;
      if (s.rover_c1_m && s.base_c1_m && (commonL1 || commonL2) && s.satellite_clock_m) ++common;
    }
    auto& result = results[epochIndex];
    result.observation.gps_seconds = epoch.gps_seconds;
    if (common < impl_->config.minimum_satellites || obs.size() < 2 * static_cast<std::size_t>(impl_->config.minimum_satellites)) {
      result.diagnostic = "insufficient common L1/code satellites"; continue;
    }
    const int ok = rtkpos(&rtk, obs.data(), static_cast<int>(obs.size()), &nav);
    const int status = rtk.sol.stat;
    if (!ok || (status != SOLQ_FLOAT && status != SOLQ_FIX) || (status == SOLQ_FLOAT && !impl_->config.accept_float) || (status == SOLQ_FIX && !impl_->config.accept_fix)) {
      result.diagnostic = "RTKLIB status=" + std::to_string(status) + ", obs=" + std::to_string(obs.size()) + ", common=" + std::to_string(common);
      if (epochIndex == 0) {
        sol_t pointSolution{};
        for (int axis = 0; axis < 3; ++axis) pointSolution.rr[axis] = impl_->config.initial_rover_position_ecef_m[axis];
        char message[128] = {};
        const int pointOk = pntpos(obs.data(), static_cast<int>(obs.size() / 2), &nav, &options, &pointSolution, nullptr, nullptr, message);
        int satelliteStatesOk = 0;
        for (std::size_t i = 0; i < obs.size() / 2; ++i) {
          double rs[6] = {}, dts[2] = {}, variance = 0.0; int health = 0;
          const gtime_t transmit = timeadd(time, -obs[i].P[0] / CLIGHT);
          satelliteStatesOk += satpos(transmit, time, obs[i].sat, EPHOPT_PREC, &nav, rs, dts, &variance, &health);
        }
        result.diagnostic += ", SPP=" + std::to_string(pointOk) + ":" + message + ", precise_states=" + std::to_string(satelliteStatesOk);
      }
      if (epochIndex == 0 && rtk.errbuf[0]) result.diagnostic += ", detail=" + std::string(rtk.errbuf);
      continue;
    }
    const bool fixed = status == SOLQ_FIX;
    const double scale = fixed ? impl_->config.fix_covariance_scale : impl_->config.float_covariance_scale;
    result.observation.solution_status = fixed ? "FIX" : "FLOAT";
    result.observation.used_satellites = rtk.sol.ns;
    result.observation.ambiguity_ratio = rtk.sol.ratio;
    for (int axis = 0; axis < 3; ++axis) result.observation.antenna_position_ecef_m[axis] = fixed ? rtk.xa[axis] : rtk.x[axis];
    const double* state = fixed ? rtk.xa : rtk.x;
    const double* covariance = fixed ? rtk.Pa : rtk.P;
    const int stride = fixed ? rtk.na : rtk.nx;
    bool validPositionCovariance = true;
    for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
      result.observation.covariance(row, col) = covariance[row + col * stride] * scale;
    for (int axis = 0; axis < 3; ++axis) {
      const double variance = result.observation.covariance(axis, axis);
      validPositionCovariance = validPositionCovariance && std::isfinite(variance) && variance > 0.0 && variance <= impl_->config.maximum_position_variance_m2;
    }
    if (!validPositionCovariance) { result.diagnostic = "invalid or excessive RTK position covariance"; continue; }
    result.observation.valid_components = 0x07;
    if (rtk.opt.dynamics && stride >= 6) {
      for (int axis = 0; axis < 3; ++axis) result.observation.antenna_velocity_ecef_mps[axis] = state[axis + 3];
      bool validVelocityCovariance = true;
      for (int row = 0; row < 6; ++row) for (int col = 0; col < 6; ++col)
        result.observation.covariance(row, col) = covariance[row + col * stride] * scale;
      for (int axis = 3; axis < 6; ++axis) {
        const double variance = result.observation.covariance(axis, axis);
        validVelocityCovariance = validVelocityCovariance && std::isfinite(variance) && variance > 0.0 && variance <= impl_->config.maximum_velocity_variance_m2ps2;
      }
      if (validVelocityCovariance) result.observation.valid_components |= 0x38;
      else result.observation.antenna_velocity_ecef_mps.setZero();
    }
    result.valid = true;
  }
  rtkfree(&rtk);
  return results;
}
}
