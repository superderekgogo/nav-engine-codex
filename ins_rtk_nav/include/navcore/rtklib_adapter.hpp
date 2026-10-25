#pragma once

#include "navcore/messages.hpp"
#include <string>
#include <vector>

namespace navcore {
struct RtkAdapterConfig {
  int nav_system = 1; // RTKLIB SYS_GPS
  bool accept_fix = true;
  bool accept_float = true;
  int minimum_satellites = 5;
  int frequency_count = 2;
  bool use_l2_pseudorange = false;
  std::string ambiguity_mode = "continuous";
  double ambiguity_ratio_threshold = 3.0;
  int minimum_lock_count = 10;
  int minimum_fix_count = 10;
  Eigen::Vector3d initial_rover_position_ecef_m = Eigen::Vector3d::Zero();
  double float_covariance_scale = 1.0;
  double fix_covariance_scale = 1.0;
  double maximum_position_variance_m2 = 100.0;
  double maximum_velocity_variance_m2ps2 = 25.0;
};

struct RtkEpochResult {
  RtkPositionVelocityObservation observation;
  bool valid = false;
  std::string diagnostic;
};

class RtklibAdapter {
public:
  explicit RtklibAdapter(RtkAdapterConfig config);
  ~RtklibAdapter();
  RtklibAdapter(const RtklibAdapter&) = delete;
  RtklibAdapter& operator=(const RtklibAdapter&) = delete;
  std::vector<RtkEpochResult> solve(const std::vector<PreparedGnssEpoch>& epochs);
private:
  struct Impl;
  Impl* impl_;
};
}
