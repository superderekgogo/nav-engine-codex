#include "nav/gmis/gmis_lc_models.hpp"

#include <cmath>
#include <stdexcept>

namespace nav::gmis {
namespace {
using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;
constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84E2 = 0.0066943799901413165;
constexpr double kEarthRate = 7.2921151467e-5;

void checkState(const Eigen::VectorXd& state) {
  if (state.size() != StateIndex::dimension || !state.allFinite())
    throw std::invalid_argument("GMIS state must be finite and exactly 21-dimensional");
}

void checkPositiveDt(double dt) {
  if (!std::isfinite(dt) || dt <= 0.0)
    throw std::invalid_argument("GMIS prediction dt must be finite and positive");
}

Matrix3 skew(const Vector3& v) {
  Matrix3 result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

void radiiOfCurvature(double latitude, double height, double& meridian, double& prime_vertical) {
  const double sin_lat = std::sin(latitude);
  const double w = std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  prime_vertical = kWgs84A / w;
  meridian = kWgs84A * (1.0 - kWgs84E2) / (w * w * w);
  (void)height;
}

void validateOrigin(const GeodeticOrigin& origin) {
  if (!std::isfinite(origin.latitude_rad) || !std::isfinite(origin.longitude_rad) ||
      !std::isfinite(origin.height_m) || std::abs(origin.latitude_rad) > 0.5 * kPi)
    throw std::invalid_argument("GMIS geodetic origin is invalid");
}

struct EarthRateJacobians {
  Matrix3 d_omega_in_d_position = Matrix3::Zero();
  Matrix3 d_omega_in_d_velocity = Matrix3::Zero();
  Matrix3 d_omega_coriolis_d_position = Matrix3::Zero();
  Matrix3 d_omega_coriolis_d_velocity = Matrix3::Zero();
  Matrix3 d_gravity_d_position = Matrix3::Zero();
};

EarthRateJacobians earthRateJacobians(const Vector3& position, const Vector3& velocity,
                                     const GeodeticOrigin& origin) {
  EarthRateJacobians j;
  for (int axis = 0; axis < 3; ++axis) {
    const double dp = axis == 2 ? 0.5 : 0.5;
    Vector3 plus = position, minus = position;
    plus(axis) += dp;
    minus(axis) -= dp;
    const auto tp = GmisLcModels::earthNavigationTerms(plus, velocity, origin);
    const auto tm = GmisLcModels::earthNavigationTerms(minus, velocity, origin);
    const Vector3 omega_in_p = tp.earth_rate_n_radps + tp.transport_rate_n_radps;
    const Vector3 omega_in_m = tm.earth_rate_n_radps + tm.transport_rate_n_radps;
    const Vector3 omega_c_p = 2.0 * tp.earth_rate_n_radps + tp.transport_rate_n_radps;
    const Vector3 omega_c_m = 2.0 * tm.earth_rate_n_radps + tm.transport_rate_n_radps;
    j.d_omega_in_d_position.col(axis) = (omega_in_p - omega_in_m) / (2.0 * dp);
    j.d_omega_coriolis_d_position.col(axis) = (omega_c_p - omega_c_m) / (2.0 * dp);
    j.d_gravity_d_position.col(axis) = (tp.gravity_n_mps2 - tm.gravity_n_mps2) / (2.0 * dp);

    const double dv = 1e-3;
    Vector3 v_plus = velocity, v_minus = velocity;
    v_plus(axis) += dv;
    v_minus(axis) -= dv;
    const auto tvp = GmisLcModels::earthNavigationTerms(position, v_plus, origin);
    const auto tvm = GmisLcModels::earthNavigationTerms(position, v_minus, origin);
    j.d_omega_in_d_velocity.col(axis) =
        (tvp.transport_rate_n_radps - tvm.transport_rate_n_radps) / (2.0 * dv);
    j.d_omega_coriolis_d_velocity.col(axis) =
        (tvp.transport_rate_n_radps - tvm.transport_rate_n_radps) / (2.0 * dv);
  }
  return j;
}

void addBlock(Eigen::MatrixXd& target, int offset, const Matrix3& block) {
  target.block<3, 3>(offset, offset) += block;
}
} // namespace

Matrix3 GmisLcModels::cNedToBody(const Vector3& angles) {
  const double roll = angles.x(), pitch = angles.y(), heading = angles.z();
  const double sr = std::sin(roll), cr = std::cos(roll);
  const double sp = std::sin(pitch), cp = std::cos(pitch);
  const double sh = std::sin(heading), ch = std::cos(heading);
  Matrix3 c;
  // C_body_to_ned = Rz(heading) * Ry(pitch) * Rx(roll); return its transpose.
  c << ch * cp,                   sh * cp,                  -sp,
       ch * sp * sr - sh * cr,    sh * sp * sr + ch * cr,   cp * sr,
       ch * sp * cr + sh * sr,    sh * sp * cr - ch * sr,   cp * cr;
  return c;
}

std::array<Matrix3, 3> GmisLcModels::cNedToBodyPartials(const Vector3& a) {
  const double r = a.x(), p = a.y(), h = a.z();
  const double sr = std::sin(r), cr = std::cos(r);
  const double sp = std::sin(p), cp = std::cos(p);
  const double sh = std::sin(h), ch = std::cos(h);
  std::array<Matrix3, 3> d;
  d[0] << 0.0, 0.0, 0.0,
          ch * sp * cr + sh * sr, sh * sp * cr - ch * sr, cp * cr,
          -ch * sp * sr + sh * cr, -sh * sp * sr - ch * cr, -cp * sr;
  d[1] << -ch * sp, -sh * sp, -cp,
          ch * cp * sr, sh * cp * sr, -sp * sr,
          ch * cp * cr, sh * cp * cr, -sp * cr;
  d[2] << -sh * cp, ch * cp, 0.0,
          -sh * sp * sr - ch * cr, ch * sp * sr - sh * cr, 0.0,
          -sh * sp * cr + ch * sr, ch * sp * cr + sh * sr, 0.0;
  return d;
}

Matrix3 GmisLcModels::eulerRateMatrix(double pitch, double roll) {
  const double cp = std::cos(pitch);
  if (std::abs(cp) < 1e-4)
    throw std::domain_error("GMIS Euler attitude is too close to pitch singularity");
  const double sr = std::sin(roll), cr = std::cos(roll);
  const double tp = std::tan(pitch), sec = 1.0 / cp;
  Matrix3 m;
  m << 1.0, sr * tp, cr * tp,
       0.0, cr,      -sr,
       0.0, sr * sec, cr * sec;
  return m;
}

EarthNavigationTerms GmisLcModels::earthNavigationTerms(
    const Vector3& position_ned, const Vector3& velocity_ned,
    const GeodeticOrigin& origin) {
  validateOrigin(origin);
  if (!position_ned.allFinite() || !velocity_ned.allFinite())
    throw std::invalid_argument("GMIS position/velocity for Earth model must be finite");

  double rm0 = 0.0, rn0 = 0.0;
  radiiOfCurvature(origin.latitude_rad, origin.height_m, rm0, rn0);
  const double latitude = origin.latitude_rad + position_ned.x() / (rm0 + origin.height_m);
  const double longitude = origin.longitude_rad +
      position_ned.y() / ((rn0 + origin.height_m) * std::cos(origin.latitude_rad));
  const double height = origin.height_m - position_ned.z();
  if (std::abs(latitude) >= 0.5 * kPi)
    throw std::domain_error("GMIS local-NED Earth model crossed a polar singularity");

  double rm = 0.0, rn = 0.0;
  radiiOfCurvature(latitude, height, rm, rn);
  const double sin_lat = std::sin(latitude), cos_lat = std::cos(latitude);
  EarthNavigationTerms terms;
  terms.earth_rate_n_radps = {kEarthRate * cos_lat, 0.0, -kEarthRate * sin_lat};
  terms.transport_rate_n_radps = {
      velocity_ned.y() / (rn + height),
      -velocity_ned.x() / (rm + height),
      -velocity_ned.y() * std::tan(latitude) / (rn + height)};
  const double sin2 = sin_lat * sin_lat;
  const double normal_gravity = 9.7803253359 * (1.0 + 0.00193185265241 * sin2) /
                                std::sqrt(1.0 - kWgs84E2 * sin2);
  const double gravity = normal_gravity - 3.086e-6 * height;
  terms.gravity_n_mps2 = {0.0, 0.0, gravity};
  (void)longitude; // NED transport and gravity are longitude-independent on WGS-84.
  return terms;
}

Eigen::Matrix<double, 6, 1> GmisLcModels::predictEarthAwareImu(
    const Eigen::VectorXd& x, const ImuSample& sample, const GeodeticOrigin& origin) {
  checkState(x);
  validateOrigin(origin);
  if (!std::isfinite(sample.time_seconds) || !sample.gyro_radps.allFinite() ||
      !sample.specific_force_mps2.allFinite())
    throw std::invalid_argument("GMIS Earth-aware IMU sample is invalid");
  if (sample.gyro_rate_convention != ImuSample::GyroRateConvention::BodyRelativeInertial)
    throw std::invalid_argument("GMIS Earth-aware IMU model requires raw body-relative-inertial gyro rate");

  const Vector3 position = x.segment<3>(StateIndex::position);
  const Vector3 velocity = x.segment<3>(StateIndex::velocity);
  const Vector3 acceleration = x.segment<3>(StateIndex::acceleration);
  const Vector3 angles = x.segment<3>(StateIndex::attitude);
  const Vector3 omega_nb = x.segment<3>(StateIndex::angular_rate);
  const auto earth = earthNavigationTerms(position, velocity, origin);
  const Vector3 omega_in_n = earth.earth_rate_n_radps + earth.transport_rate_n_radps;
  const Vector3 omega_coriolis_n = 2.0 * earth.earth_rate_n_radps + earth.transport_rate_n_radps;
  const Matrix3 c_n_to_b = cNedToBody(angles);
  const Vector3 omega_ib_b = omega_nb + c_n_to_b * omega_in_n;
  const Vector3 f_n = acceleration - earth.gravity_n_mps2 + omega_coriolis_n.cross(velocity);
  const Vector3 f_b = c_n_to_b * f_n;
  const Vector3 bg = x.segment<3>(StateIndex::gyro_bias);
  const Vector3 ba = x.segment<3>(StateIndex::accel_bias);

  Eigen::Matrix<double, 6, 1> predicted;
  predicted.head<3>() = omega_ib_b + bg;
  predicted.tail<3>() = f_b + ba;
  return predicted;
}

Prediction GmisLcModels::predict(const Eigen::VectorXd& x, double dt,
                                 const GmisProcessNoise& noise) {
  checkState(x);
  checkPositiveDt(dt);
  if (!noise.jerk_variance.allFinite() || (noise.jerk_variance.array() < 0.0).any() ||
      !noise.angular_acceleration_variance.allFinite() || (noise.angular_acceleration_variance.array() < 0.0).any() ||
      !noise.gyro_bias_rw_psd.allFinite() || (noise.gyro_bias_rw_psd.array() < 0.0).any() ||
      !noise.accel_bias_rw_psd.allFinite() || (noise.accel_bias_rw_psd.array() < 0.0).any())
    throw std::invalid_argument("GMIS process noise values must be finite and non-negative");

  Prediction result;
  result.state = x;
  result.jacobian = Eigen::MatrixXd::Identity(StateIndex::dimension, StateIndex::dimension);
  result.process_noise = Eigen::MatrixXd::Zero(StateIndex::dimension, StateIndex::dimension);
  const Matrix3 identity = Matrix3::Identity();
  const double dt2 = dt * dt;

  result.state.segment<3>(StateIndex::position) +=
      dt * x.segment<3>(StateIndex::velocity) + 0.5 * dt2 * x.segment<3>(StateIndex::acceleration);
  result.state.segment<3>(StateIndex::velocity) += dt * x.segment<3>(StateIndex::acceleration);
  result.jacobian.block<3, 3>(StateIndex::position, StateIndex::velocity) = dt * identity;
  result.jacobian.block<3, 3>(StateIndex::position, StateIndex::acceleration) = 0.5 * dt2 * identity;
  result.jacobian.block<3, 3>(StateIndex::velocity, StateIndex::acceleration) = dt * identity;

  const Vector3 angles = x.segment<3>(StateIndex::attitude);
  const Vector3 omega = x.segment<3>(StateIndex::angular_rate);
  const Matrix3 m = eulerRateMatrix(angles.y(), angles.x());
  const auto dm = [&]() {
    std::array<Matrix3, 2> out;
    const double p = angles.y(), r = angles.x();
    const double cp = std::cos(p), tp = std::tan(p), sec = 1.0 / cp;
    const double sr = std::sin(r), cr = std::cos(r);
    out[0] << 0.0, cr * tp, -sr * tp,
              0.0, -sr,     -cr,
              0.0, cr * sec, -sr * sec;
    out[1] << 0.0, sr * sec * sec, cr * sec * sec,
              0.0, 0.0,             0.0,
              0.0, sr * sec * tp,   cr * sec * tp;
    return out;
  }();
  result.state.segment<3>(StateIndex::attitude) = normalizeEuler(angles + dt * m * omega);
  result.jacobian.block<3, 3>(StateIndex::attitude, StateIndex::angular_rate) = dt * m;
  result.jacobian.block<3, 1>(StateIndex::attitude, StateIndex::attitude) += dt * dm[0] * omega;
  result.jacobian.block<3, 1>(StateIndex::attitude, StateIndex::attitude + 1) += dt * dm[1] * omega;

  Eigen::Matrix<double, 9, 3> gammaPosition = Eigen::Matrix<double, 9, 3>::Zero();
  gammaPosition.block<3, 3>(0, 0) = (dt2 * dt / 6.0) * identity;
  gammaPosition.block<3, 3>(3, 0) = (dt2 / 2.0) * identity;
  gammaPosition.block<3, 3>(6, 0) = dt * identity;
  result.process_noise.block<9, 9>(StateIndex::position, StateIndex::position) =
      gammaPosition * noise.jerk_variance.asDiagonal() * gammaPosition.transpose();

  Eigen::Matrix<double, 6, 3> gammaAngular = Eigen::Matrix<double, 6, 3>::Zero();
  gammaAngular.block<3, 3>(0, 0) = 0.5 * dt2 * m;
  gammaAngular.block<3, 3>(3, 0) = dt * identity;
  result.process_noise.block<6, 6>(StateIndex::attitude, StateIndex::attitude) =
      gammaAngular * noise.angular_acceleration_variance.asDiagonal() * gammaAngular.transpose();

  addBlock(result.process_noise, StateIndex::gyro_bias, dt * noise.gyro_bias_rw_psd.asDiagonal());
  addBlock(result.process_noise, StateIndex::accel_bias, dt * noise.accel_bias_rw_psd.asDiagonal());
  return result;
}

LinearizedObservation GmisLcModels::linearizeImu(
    const Eigen::VectorXd& x, const ImuSample& sample,
    const Eigen::Matrix<double, 6, 6>& covariance, const Vector3& gravity) {
  checkState(x);
  if (!sample.gyro_radps.allFinite() || !sample.specific_force_mps2.allFinite() ||
      !covariance.allFinite() || !gravity.allFinite())
    throw std::invalid_argument("GMIS IMU observation contains non-finite values");
  if (sample.gyro_rate_convention != ImuSample::GyroRateConvention::BodyRelativeNavigation)
    throw std::invalid_argument("GMIS phase-one IMU model requires body-relative-navigation rate; raw inertial gyro needs an Earth-aware model");

  const Vector3 angles = x.segment<3>(StateIndex::attitude);
  const Vector3 omega = x.segment<3>(StateIndex::angular_rate);
  const Vector3 bg = x.segment<3>(StateIndex::gyro_bias);
  const Vector3 ba = x.segment<3>(StateIndex::accel_bias);
  const Matrix3 c = cNedToBody(angles);
  const Vector3 netAcceleration = x.segment<3>(StateIndex::acceleration) - gravity;
  const Vector3 idealSpecificForce = c * netAcceleration;

  LinearizedObservation out;
  out.innovation.resize(6);
  out.jacobian = Eigen::MatrixXd::Zero(6, StateIndex::dimension);
  out.noise = covariance;
  out.innovation.head<3>() = sample.gyro_radps - (omega + bg);
  out.innovation.tail<3>() = sample.specific_force_mps2 -
                                  (idealSpecificForce + ba);
  out.jacobian.block<3, 3>(0, StateIndex::angular_rate) = Matrix3::Identity();
  out.jacobian.block<3, 3>(0, StateIndex::gyro_bias) = Matrix3::Identity();
  out.jacobian.block<3, 3>(3, StateIndex::acceleration) = c;
  const auto dC = cNedToBodyPartials(angles);
  for (int j = 0; j < 3; ++j)
    out.jacobian.block<3, 1>(3, StateIndex::attitude + j) = dC[j] * netAcceleration;
  out.jacobian.block<3, 3>(3, StateIndex::accel_bias) = Matrix3::Identity();
  return out;
}

LinearizedObservation GmisLcModels::linearizeImuEarthAware(
    const Eigen::VectorXd& x, const ImuSample& sample,
    const Eigen::Matrix<double, 6, 6>& covariance,
    const GeodeticOrigin& origin) {
  checkState(x);
  if (!covariance.allFinite())
    throw std::invalid_argument("GMIS Earth-aware IMU covariance contains non-finite values");
  const Eigen::Matrix<double, 6, 1> predicted = predictEarthAwareImu(x, sample, origin);

  const Vector3 position = x.segment<3>(StateIndex::position);
  const Vector3 velocity = x.segment<3>(StateIndex::velocity);
  const Vector3 acceleration = x.segment<3>(StateIndex::acceleration);
  const Vector3 angles = x.segment<3>(StateIndex::attitude);
  const Vector3 omega_nb = x.segment<3>(StateIndex::angular_rate);
  const Matrix3 c = cNedToBody(angles);
  const auto d_c = cNedToBodyPartials(angles);
  const auto earth = earthNavigationTerms(position, velocity, origin);
  const Vector3 omega_in = earth.earth_rate_n_radps + earth.transport_rate_n_radps;
  const Vector3 omega_coriolis = 2.0 * earth.earth_rate_n_radps + earth.transport_rate_n_radps;
  const Vector3 f_n = acceleration - earth.gravity_n_mps2 + omega_coriolis.cross(velocity);
  const Vector3 f_b = c * f_n;
  const EarthRateJacobians earth_j = earthRateJacobians(position, velocity, origin);

  LinearizedObservation out;
  out.innovation.resize(6);
  out.innovation.head<3>() = sample.gyro_radps - predicted.head<3>();
  out.innovation.tail<3>() = sample.specific_force_mps2 - predicted.tail<3>();
  out.jacobian = Eigen::MatrixXd::Zero(6, StateIndex::dimension);
  out.noise = covariance;

  out.jacobian.block<3, 3>(0, StateIndex::position) = c * earth_j.d_omega_in_d_position;
  out.jacobian.block<3, 3>(0, StateIndex::velocity) = c * earth_j.d_omega_in_d_velocity;
  for (int axis = 0; axis < 3; ++axis)
    out.jacobian.block<3, 1>(0, StateIndex::attitude + axis) = d_c[axis] * omega_in;
  out.jacobian.block<3, 3>(0, StateIndex::angular_rate) = Matrix3::Identity();
  out.jacobian.block<3, 3>(0, StateIndex::gyro_bias) = Matrix3::Identity();

  const Matrix3 q_position = -earth_j.d_gravity_d_position -
                             skew(velocity) * earth_j.d_omega_coriolis_d_position;
  const Matrix3 q_velocity = skew(omega_coriolis) -
                             skew(velocity) * earth_j.d_omega_coriolis_d_velocity;
  out.jacobian.block<3, 3>(3, StateIndex::position) = c * q_position;
  out.jacobian.block<3, 3>(3, StateIndex::velocity) = c * q_velocity;
  out.jacobian.block<3, 3>(3, StateIndex::acceleration) = c;
  for (int axis = 0; axis < 3; ++axis)
    out.jacobian.block<3, 1>(3, StateIndex::attitude + axis) = d_c[axis] * f_n;
  out.jacobian.block<3, 3>(3, StateIndex::accel_bias) = Matrix3::Identity();
  return out;
}

LinearizedObservation GmisLcModels::linearizeRtkPositionVelocity(
    const Eigen::VectorXd& x, const Vector3& position, const Vector3& velocity,
    const Eigen::Matrix<double, 6, 6>& covariance,
    const Vector3& lever_arm_body_m) {
  checkState(x);
  if (!position.allFinite() || !velocity.allFinite() || !covariance.allFinite() ||
      !lever_arm_body_m.allFinite())
    throw std::invalid_argument("GMIS RTK observation contains non-finite values");
  const Vector3 angles = x.segment<3>(StateIndex::attitude);
  const Vector3 omega_nb = x.segment<3>(StateIndex::angular_rate);
  const Matrix3 c_n_to_b = cNedToBody(angles);
  const Matrix3 c_b_to_n = c_n_to_b.transpose();
  const auto d_c_n_to_b = cNedToBodyPartials(angles);
  const Vector3 rotational_velocity_b = omega_nb.cross(lever_arm_body_m);
  LinearizedObservation out;
  out.innovation.resize(6);
  out.jacobian = Eigen::MatrixXd::Zero(6, StateIndex::dimension);
  out.noise = covariance;
  out.innovation.head<3>() = position -
      (x.segment<3>(StateIndex::position) + c_b_to_n * lever_arm_body_m);
  out.innovation.tail<3>() = velocity -
      (x.segment<3>(StateIndex::velocity) + c_b_to_n * rotational_velocity_b);
  out.jacobian.block<3, 3>(0, StateIndex::position) = Matrix3::Identity();
  out.jacobian.block<3, 3>(3, StateIndex::velocity) = Matrix3::Identity();
  for (int axis = 0; axis < 3; ++axis) {
    out.jacobian.block<3, 1>(0, StateIndex::attitude + axis) =
        d_c_n_to_b[axis].transpose() * lever_arm_body_m;
    out.jacobian.block<3, 1>(3, StateIndex::attitude + axis) =
        d_c_n_to_b[axis].transpose() * rotational_velocity_b;
  }
  out.jacobian.block<3, 3>(3, StateIndex::angular_rate) =
      -c_b_to_n * skew(lever_arm_body_m);
  return out;
}

Vector3 GmisLcModels::normalizeEuler(const Vector3& angles) {
  constexpr double twoPi = 2.0 * 3.14159265358979323846;
  Vector3 normalized = angles;
  normalized.x() = std::remainder(normalized.x(), twoPi);
  normalized.z() = std::remainder(normalized.z(), twoPi);
  return normalized;
}

} // namespace nav::gmis
