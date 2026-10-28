#pragma once

#include "nav/gmis/gmis_lc_models.hpp"

namespace nav::gmis {

enum class GmisImuModelKind { SimplifiedNavigationRate, EarthAwareInertialRate };

struct GmisLcConfiguration {
  std::string algorithm = "gmis";
  std::string combination_mode = "gmis_lc";
  GmisImuModelKind imu_model = GmisImuModelKind::EarthAwareInertialRate;
  GeodeticOrigin origin;
  Eigen::Vector3d antenna_lever_arm_body_m = Eigen::Vector3d::Zero();
  GmisProcessNoise process_noise;
  Eigen::Matrix<double, 6, 6> imu_measurement_covariance =
      Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> rtk_measurement_covariance =
      Eigen::Matrix<double, 6, 6>::Identity();

  static GmisLcConfiguration fromJson(const navcore::Json& json);
};

} // namespace nav::gmis
