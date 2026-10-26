#include "navigation/strapdown_mechanizer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation {
namespace {
constexpr double kA = 6378137.0;
constexpr double kE2 = 0.0066943799901413165;
constexpr double kOmegaIe = 7.2921151467e-5;
constexpr double kGravity = 9.80665;

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Quaterniond rotationVector(const Eigen::Vector3d& v) {
  const double angle = v.norm();
  if (angle < 1e-12) return Eigen::Quaterniond(1.0, v.x() * 0.5, v.y() * 0.5, v.z() * 0.5).normalized();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, v / angle));
}

Eigen::Quaterniond eulerNedToQuaternion(const Eigen::Vector3d& rpy) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
}

void radii(double latitude, double height, double& rm, double& rn) {
  const double s = std::sin(latitude);
  const double d = std::sqrt(1.0 - kE2 * s * s);
  rn = kA / d;
  rm = kA * (1.0 - kE2) / (d * d * d);
  (void)height;
}
}

Eigen::Vector3d ecefToLla(const Eigen::Vector3d& ecef) {
  const double x = ecef.x(), y = ecef.y(), z = ecef.z();
  const double p = std::hypot(x, y);
  if (!ecef.allFinite() || p < 1e-6) throw std::runtime_error("invalid rover ECEF coordinate");
  const double lon = std::atan2(y, x);
  double lat = std::atan2(z, p * (1.0 - kE2));
  double h = 0.0;
  for (int i = 0; i < 12; ++i) {
    const double s = std::sin(lat), n = kA / std::sqrt(1.0 - kE2 * s * s);
    h = p / std::cos(lat) - n;
    const double next = std::atan2(z, p * (1.0 - kE2 * n / (n + h)));
    if (std::abs(next - lat) < 1e-13) { lat = next; break; }
    lat = next;
  }
  return {lat, lon, h};
}

Eigen::Vector3d llaToEcef(const Eigen::Vector3d& lla) {
  const double lat = lla.x(), lon = lla.y(), h = lla.z();
  const double s = std::sin(lat), c = std::cos(lat), n = kA / std::sqrt(1.0 - kE2 * s * s);
  return {(n + h) * c * std::cos(lon), (n + h) * c * std::sin(lon), (n * (1.0 - kE2) + h) * s};
}

StrapdownMechanizer::StrapdownMechanizer(const InsInitialState& initial)
    : gyro_bias_(initial.gyro_bias_radps), accel_bias_(initial.accel_bias_mps2) {
  state_.lla_rad_m = initial.lla_rad_m;
  state_.ecef_m = llaToEcef(state_.lla_rad_m);
  state_.velocity_ned_mps = initial.velocity_ned_mps;
  state_.q_body_to_ned = eulerNedToQuaternion(initial.euler_rpy_rad).normalized();
  state_.gyro_bias_radps = gyro_bias_;
  state_.accel_bias_mps2 = accel_bias_;
}

std::optional<navcore::NavigationSolution> StrapdownMechanizer::update(const navcore::ImuSample& sample) {
  if (!previous_sample_) {
    previous_sample_ = sample;
    state_.gps_seconds = sample.gps_seconds;
    return state_;
  }
  const double dt = sample.gps_seconds - previous_sample_->gps_seconds;
  if (!std::isfinite(dt) || dt <= 0.0) throw std::runtime_error("IMU timestamps must be strictly increasing");
  if (dt > 1.0) throw std::runtime_error("IMU gap exceeds 1 second; check timestamps before INS propagation");

  const Eigen::Vector3d delta_angle_raw = 0.5 * (previous_sample_->gyro_radps + sample.gyro_radps - 2.0 * gyro_bias_) * dt;
  const Eigen::Vector3d delta_velocity_raw = 0.5 * (previous_sample_->specific_force_mps2 + sample.specific_force_mps2 - 2.0 * accel_bias_) * dt;
  Eigen::Vector3d delta_angle = delta_angle_raw;
  Eigen::Vector3d delta_velocity = delta_velocity_raw;
  if (have_previous_increment_) {
    delta_angle += previous_delta_angle_.cross(delta_angle_raw) / 12.0;
    delta_velocity += 0.5 * delta_angle_raw.cross(delta_velocity_raw) +
                      (previous_delta_angle_.cross(delta_velocity_raw) + previous_delta_velocity_.cross(delta_angle_raw)) / 12.0;
  }

  const double lat = state_.lla_rad_m.x(), height = state_.lla_rad_m.z();
  double rm = 0.0, rn = 0.0;
  radii(lat, height, rm, rn);
  const auto& v = state_.velocity_ned_mps;
  const Eigen::Vector3d omega_ie(kOmegaIe * std::cos(lat), 0.0, -kOmegaIe * std::sin(lat));
  const Eigen::Vector3d omega_en(v.y() / (rn + height), -v.x() / (rm + height), -v.y() * std::tan(lat) / (rn + height));
  const Eigen::Vector3d nav_rotation = (omega_ie + omega_en) * dt;
  const Eigen::Matrix3d c_bn = state_.q_body_to_ned.toRotationMatrix();
  const Eigen::Vector3d delta_v_n = (Eigen::Matrix3d::Identity() - 0.5 * skew(nav_rotation)) * c_bn * delta_velocity;
  const Eigen::Vector3d gravity_n(0.0, 0.0, kGravity);
  const Eigen::Vector3d delta_v_gravity = (gravity_n - (2.0 * omega_ie + omega_en).cross(v)) * dt;
  const Eigen::Vector3d new_velocity = v + delta_v_n + delta_v_gravity;
  const Eigen::Vector3d midpoint_velocity = 0.5 * (v + new_velocity);
  const double mid_lat = lat + midpoint_velocity.x() * dt / (rm + height);
  const double mid_height = height - midpoint_velocity.z() * dt;
  const double mid_lon = state_.lla_rad_m.y() + midpoint_velocity.y() * dt / ((rn + height) * std::cos(lat));

  const Eigen::Quaterniond q_new = (rotationVector(-nav_rotation) * state_.q_body_to_ned * rotationVector(delta_angle)).normalized();
  state_.gps_seconds = sample.gps_seconds;
  state_.velocity_ned_mps = new_velocity;
  state_.lla_rad_m = {mid_lat, mid_lon, mid_height};
  state_.ecef_m = llaToEcef(state_.lla_rad_m);
  state_.q_body_to_ned = q_new;
  previous_sample_ = sample;
  previous_delta_angle_ = delta_angle_raw;
  previous_delta_velocity_ = delta_velocity_raw;
  have_previous_increment_ = true;
  return state_;
}
} // namespace navigation
