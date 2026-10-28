#include "nav/gmis/gmis_lc_config.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace nav::gmis {
namespace {
Eigen::Vector3d vector3(const navcore::Json& object, const char* key) {
  const auto& values = object.at(key).asArray();
  if (values.size() != 3) throw std::invalid_argument(std::string("config vector must have 3 values: ") + key);
  Eigen::Vector3d result;
  for (int i = 0; i < 3; ++i) {
    if (!values[static_cast<std::size_t>(i)].isNumber())
      throw std::invalid_argument(std::string("config vector has a non-number value: ") + key);
    result(i) = values[static_cast<std::size_t>(i)].asNumber();
    if (!std::isfinite(result(i))) throw std::invalid_argument(std::string("config vector is not finite: ") + key);
  }
  return result;
}

void requireString(const navcore::Json& object, const char* key, const char* expected) {
  const std::string actual = object.at(key).asString();
  if (actual != expected)
    throw std::invalid_argument(std::string("unsupported GMIS configuration ") + key + "='" + actual + "' (expected '" + expected + "')");
}

void requireBool(const navcore::Json& object, const char* key, bool expected) {
  if (object.at(key).asBool() != expected)
    throw std::invalid_argument(std::string("unsupported GMIS configuration boolean: ") + key);
}

Eigen::Matrix<double, 6, 6> diagonalVariance(const Eigen::Vector3d& first_sigma,
                                             const Eigen::Vector3d& second_sigma,
                                             const char* name) {
  if ((first_sigma.array() <= 0.0).any() || (second_sigma.array() <= 0.0).any())
    throw std::invalid_argument(std::string(name) + " standard deviations must be positive");
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  covariance.diagonal().head<3>() = first_sigma.array().square().matrix();
  covariance.diagonal().tail<3>() = second_sigma.array().square().matrix();
  return covariance;
}
} // namespace

GmisLcConfiguration GmisLcConfiguration::fromJson(const navcore::Json& json) {
  if (!json.isObject()) throw std::invalid_argument("GMIS configuration root must be an object");
  requireString(json, "algorithm", "gmis");
  requireString(json, "combination_mode", "gmis_lc");

  const auto& modules = json.at("modules");
  requireString(modules, "filter_manager", "gmis_filter_manager_v1");
  requireString(modules, "estimation", "gmis_estimation_module_v1");
  requireString(modules, "state_estimator", "generic_ekf_v1");

  const auto& frames = json.at("frames");
  requireString(frames, "navigation", "NED");
  requireString(frames, "body", "FRD");
  requireString(frames, "attitude_parameterization", "euler_zyx_roll_pitch_heading");
  requireString(json, "state_layout", "gmis_single_imu_euler_21d_fixed_lever_v1");

  const auto& imu = json.at("imu_model");
  const std::string model_id = imu.at("model_id").asString();
  GmisLcConfiguration config;
  if (model_id == "gmis_imu_earth_aware_v1") {
    config.imu_model = GmisImuModelKind::EarthAwareInertialRate;
    requireString(imu, "gyro_rate_convention", "body_relative_inertial");
    requireString(imu, "accelerometer_input", "specific_force_mps2");
    requireString(imu, "earth_rotation_and_transport_terms", "modeled");
    requireBool(imu, "raw_body_relative_inertial_gyro_supported", true);
  } else if (model_id == "gmis_imu_simplified_rate_v1") {
    config.imu_model = GmisImuModelKind::SimplifiedNavigationRate;
    requireString(imu, "gyro_rate_convention", "body_relative_navigation");
    requireString(imu, "accelerometer_input", "specific_force_mps2");
    requireString(imu, "earth_rotation_and_transport_terms", "not_modeled");
    requireBool(imu, "raw_body_relative_inertial_gyro_supported", false);
  } else {
    throw std::invalid_argument("unsupported GMIS imu_model.model_id: " + model_id);
  }

  const auto origin = vector3(frames, "geodetic_origin_lla_rad_m");
  config.origin = {origin.x(), origin.y(), origin.z()};
  config.antenna_lever_arm_body_m = vector3(json.at("gnss"), "antenna_lever_arm_body_m");
  if (std::abs(config.origin.latitude_rad) > 0.5 * 3.14159265358979323846)
    throw std::invalid_argument("GMIS geodetic origin latitude must be within [-pi/2, pi/2]");

  const auto& model_ids = json.at("measurement_models").asArray();
  bool has_imu = false, has_position = false, has_velocity = false;
  for (const auto& item : model_ids) {
    if (!item.isString()) throw std::invalid_argument("measurement_models must contain strings");
    const auto& id = item.asString();
    has_imu = has_imu || id == "imu_raw_gyro_accel_v1";
    has_position = has_position || id == "rtk_position_loose_v1";
    has_velocity = has_velocity || id == "rtk_velocity_loose_v1";
  }
  if (!has_imu || !has_position || !has_velocity)
    throw std::invalid_argument("GMIS_LC requires IMU, RTK position, and RTK velocity measurement models");

  const auto& process = json.at("process_noise");
  config.process_noise.jerk_variance = vector3(process, "jerk_variance_m2ps6");
  config.process_noise.angular_acceleration_variance = vector3(process, "angular_acceleration_variance_rad2ps4");
  config.process_noise.gyro_bias_rw_psd = vector3(process, "gyro_bias_rw_psd");
  config.process_noise.accel_bias_rw_psd = vector3(process, "accel_bias_rw_psd");

  const auto& measurement_noise = json.at("measurement_noise");
  config.imu_measurement_covariance = diagonalVariance(
      vector3(measurement_noise, "gyro_sigma_radps"),
      vector3(measurement_noise, "accel_sigma_mps2"), "IMU");
  config.rtk_measurement_covariance = diagonalVariance(
      vector3(measurement_noise, "rtk_position_sigma_m"),
      vector3(measurement_noise, "rtk_velocity_sigma_mps"), "RTK");
  return config;
}

} // namespace nav::gmis
