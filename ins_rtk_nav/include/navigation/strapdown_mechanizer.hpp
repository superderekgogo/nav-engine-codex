#pragma once

#include "navcore/messages.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>

namespace navigation {

struct InsInitialState {
  Eigen::Vector3d lla_rad_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ned_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d euler_rpy_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_mps2 = Eigen::Vector3d::Zero();
};

class StrapdownMechanizer {
public:
  explicit StrapdownMechanizer(const InsInitialState& initial);
  std::optional<navcore::NavigationSolution> update(const navcore::ImuSample& sample);
  void setNavigationState(const navcore::NavigationSolution& state) { state_ = state; }
  void applyErrorStateCorrection(const navcore::NavigationSolution& state,
                                 const Eigen::Vector3d& gyro_bias_correction_radps,
                                 const Eigen::Vector3d& accel_bias_correction_mps2) {
    state_ = state;
    gyro_bias_ += gyro_bias_correction_radps;
    accel_bias_ += accel_bias_correction_mps2;
    state_.gyro_bias_radps = gyro_bias_;
    state_.accel_bias_mps2 = accel_bias_;
  }
  const Eigen::Vector3d& gyroBias() const noexcept { return gyro_bias_; }
  const Eigen::Vector3d& accelBias() const noexcept { return accel_bias_; }

private:
  navcore::NavigationSolution state_;
  Eigen::Vector3d gyro_bias_, accel_bias_;
  std::optional<navcore::ImuSample> previous_sample_;
  Eigen::Vector3d previous_delta_angle_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d previous_delta_velocity_ = Eigen::Vector3d::Zero();
  bool have_previous_increment_ = false;
};

Eigen::Vector3d ecefToLla(const Eigen::Vector3d& ecef_m);
Eigen::Vector3d llaToEcef(const Eigen::Vector3d& lla_rad_m);

} // namespace navigation
