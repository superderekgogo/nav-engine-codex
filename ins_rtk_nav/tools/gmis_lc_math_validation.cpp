#include "nav/gmis/gmis_lc_estimation.hpp"
#include "navcore/config_loader.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using nav::gmis::GmisLcModels;
using nav::gmis::GmisProcessNoise;
using nav::gmis::ImuSample;
using nav::gmis::StateIndex;

Eigen::VectorXd stateFixture() {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(StateIndex::dimension);
  x.segment<3>(StateIndex::position) << 12.0, -4.0, 83.0;
  x.segment<3>(StateIndex::velocity) << 3.2, -0.7, 0.1;
  x.segment<3>(StateIndex::acceleration) << 0.12, -0.08, 0.03;
  x.segment<3>(StateIndex::attitude) << 0.12, -0.18, 0.67;
  x.segment<3>(StateIndex::angular_rate) << 0.03, -0.02, 0.01;
  x.segment<3>(StateIndex::gyro_bias) << 1e-4, -2e-4, 3e-4;
  x.segment<3>(StateIndex::accel_bias) << 0.01, -0.02, 0.03;
  return x;
}

template <class Function>
Eigen::MatrixXd numericalJacobian(const Eigen::VectorXd& x, Function function) {
  const Eigen::VectorXd y = function(x);
  Eigen::MatrixXd jacobian(y.size(), x.size());
  constexpr double epsilon = 1e-6;
  for (Eigen::Index j = 0; j < x.size(); ++j) {
    Eigen::VectorXd plus = x, minus = x;
    plus(j) += epsilon;
    minus(j) -= epsilon;
    jacobian.col(j) = (function(plus) - function(minus)) / (2.0 * epsilon);
  }
  return jacobian;
}

template <class Function>
Eigen::MatrixXd numericalJacobian(const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& steps,
                                  Function function) {
  const Eigen::VectorXd y = function(x);
  Eigen::MatrixXd jacobian(y.size(), x.size());
  for (Eigen::Index j = 0; j < x.size(); ++j) {
    Eigen::VectorXd plus = x, minus = x;
    plus(j) += steps(j);
    minus(j) -= steps(j);
    jacobian.col(j) = (function(plus) - function(minus)) / (2.0 * steps(j));
  }
  return jacobian;
}

void requireNear(const Eigen::MatrixXd& actual, const Eigen::MatrixXd& expected,
                 double tolerance, const char* label) {
  const double error = (actual - expected).cwiseAbs().maxCoeff();
  if (!std::isfinite(error) || error > tolerance)
    throw std::runtime_error(std::string(label) + " max abs error=" + std::to_string(error));
  std::cout << label << " max_abs_error=" << error << '\n';
}
} // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::invalid_argument("usage: gmis_lc_math_validation <gmis_lc.json>");
    const auto json = navcore::ConfigLoader().load(argv[1]);
    const auto config = nav::gmis::GmisLcConfiguration::fromJson(json);
    if (config.imu_model != nav::gmis::GmisImuModelKind::EarthAwareInertialRate)
      throw std::runtime_error("GMIS profile did not select the Earth-aware IMU model");
    std::cout << "gmis_config_assembly=PASS\n";

    const Eigen::VectorXd x = stateFixture();
    GmisProcessNoise process_noise;
    process_noise.jerk_variance.setZero();
    process_noise.angular_acceleration_variance.setZero();
    process_noise.gyro_bias_rw_psd.setZero();
    process_noise.accel_bias_rw_psd.setZero();
    constexpr double dt = 0.013;

    const auto prediction = GmisLcModels::predict(x, dt, process_noise);
    const Eigen::MatrixXd numerical_f = numericalJacobian(x, [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
      return GmisLcModels::predict(state, dt, process_noise).state;
    });
    requireNear(prediction.jacobian, numerical_f, 2e-6, "transition_jacobian");

    Eigen::MatrixXd analytic_rotation_partials(9, 3);
    const auto rotation_partials = GmisLcModels::cNedToBodyPartials(x.segment<3>(StateIndex::attitude));
    for (int j = 0; j < 3; ++j)
      analytic_rotation_partials.col(j) = Eigen::Map<const Eigen::VectorXd>(rotation_partials[j].data(), 9);
    const Eigen::VectorXd angles = x.segment<3>(StateIndex::attitude);
    const Eigen::MatrixXd numerical_rotation_partials = numericalJacobian(angles, [&](const Eigen::VectorXd& a) -> Eigen::VectorXd {
      const Eigen::Matrix3d c = GmisLcModels::cNedToBody(a);
      return Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(c.data(), 9));
    });
    requireNear(analytic_rotation_partials, numerical_rotation_partials, 2e-9,
                "rotation_matrix_partials");

    ImuSample sample;
    sample.time_seconds = 10.0;
    sample.gyro_rate_convention = ImuSample::GyroRateConvention::BodyRelativeNavigation;
    sample.gyro_radps << 0.05, -0.03, 0.02;
    sample.specific_force_mps2 << 0.2, -0.1, -9.7;
    Eigen::Matrix<double, 6, 6> imu_r = Eigen::Matrix<double, 6, 6>::Identity();
    imu_r.diagonal() << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3;
    const auto imu = GmisLcModels::linearizeImu(x, sample, imu_r);
    const Eigen::MatrixXd finite_difference_imu = numericalJacobian(x, [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
      const auto residual = GmisLcModels::linearizeImu(state, sample, imu_r).innovation;
      return Eigen::VectorXd(-residual);
    });
    requireNear(imu.jacobian, finite_difference_imu, 3e-6, "imu_observation_jacobian");

    ImuSample unspecified = sample;
    unspecified.gyro_rate_convention = ImuSample::GyroRateConvention::Unspecified;
    bool rejected_unspecified = false;
    try {
      (void)GmisLcModels::linearizeImu(x, unspecified, imu_r);
    } catch (const std::invalid_argument&) {
      rejected_unspecified = true;
    }
    if (!rejected_unspecified) throw std::runtime_error("unspecified gyro convention was not rejected");

    Eigen::VectorXd stationary = Eigen::VectorXd::Zero(StateIndex::dimension);
    ImuSample raw_imu;
    raw_imu.time_seconds = 10.0;
    raw_imu.gyro_rate_convention = ImuSample::GyroRateConvention::BodyRelativeInertial;
    const auto& origin = config.origin;
    const auto earth = GmisLcModels::earthNavigationTerms(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), origin);
    raw_imu.gyro_radps = earth.earth_rate_n_radps;
    raw_imu.specific_force_mps2 = -earth.gravity_n_mps2;
    const auto earth_prediction = GmisLcModels::predictEarthAwareImu(stationary, raw_imu, origin);
    Eigen::Matrix<double, 6, 1> expected_static_imu;
    expected_static_imu.head<3>() = earth.earth_rate_n_radps;
    expected_static_imu.tail<3>() = -earth.gravity_n_mps2;
    requireNear(earth_prediction, expected_static_imu, 1e-12, "earth_aware_static_prediction");

    const auto earth_imu = GmisLcModels::linearizeImuEarthAware(stationary, raw_imu, imu_r, origin);
    Eigen::VectorXd earth_steps = Eigen::VectorXd::Constant(StateIndex::dimension, 1e-6);
    earth_steps.segment<3>(StateIndex::position).setConstant(0.5);
    earth_steps.segment<3>(StateIndex::velocity).setConstant(1e-3);
    earth_steps.segment<3>(StateIndex::acceleration).setConstant(1e-5);
    const Eigen::MatrixXd numerical_earth_h = numericalJacobian(
        stationary, earth_steps, [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
          return GmisLcModels::predictEarthAwareImu(state, raw_imu, origin);
        });
    requireNear(earth_imu.jacobian, numerical_earth_h, 2e-8, "earth_aware_imu_jacobian");

    const Eigen::VectorXd dynamic_state = stateFixture();
    ImuSample dynamic_raw = raw_imu;
    dynamic_raw.gyro_radps = GmisLcModels::predictEarthAwareImu(dynamic_state, dynamic_raw, origin).head<3>();
    dynamic_raw.specific_force_mps2 =
        GmisLcModels::predictEarthAwareImu(dynamic_state, dynamic_raw, origin).tail<3>();
    const auto dynamic_earth_imu = GmisLcModels::linearizeImuEarthAware(
        dynamic_state, dynamic_raw, imu_r, origin);
    const Eigen::MatrixXd numerical_dynamic_earth_h = numericalJacobian(
        dynamic_state, earth_steps, [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
          return GmisLcModels::predictEarthAwareImu(state, dynamic_raw, origin);
        });
    requireNear(dynamic_earth_imu.jacobian, numerical_dynamic_earth_h, 3e-8,
                "earth_aware_dynamic_imu_jacobian");

    nav::gmis::GmisLcEstimatorSession configured_session(config);
    configured_session.initialize(raw_imu.time_seconds, stationary,
        Eigen::MatrixXd::Identity(StateIndex::dimension, StateIndex::dimension));
    const auto earth_update = configured_session.updateImu(raw_imu);
    if (!earth_update.accepted) throw std::runtime_error("configured Earth-aware IMU update was rejected");
    std::cout << "earth_aware_imu_update=PASS\n";

    const Eigen::Vector3d rtk_position(12.5, -3.8, 83.2);
    const Eigen::Vector3d rtk_velocity(3.0, -0.6, 0.0);
    const Eigen::Vector3d lever_arm(0.8, -0.3, 1.2);
    const auto rtk = GmisLcModels::linearizeRtkPositionVelocity(
        x, rtk_position, rtk_velocity, Eigen::Matrix<double, 6, 6>::Identity(), lever_arm);
    const Eigen::MatrixXd numerical_h_rtk = numericalJacobian(x, [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
      return Eigen::VectorXd(-GmisLcModels::linearizeRtkPositionVelocity(
          state, rtk_position, rtk_velocity, Eigen::Matrix<double, 6, 6>::Identity(), lever_arm).innovation);
    });
    requireNear(rtk.jacobian, numerical_h_rtk, 1e-7, "rtk_observation_jacobian_with_fixed_lever_arm");

    nav::gmis::GmisLcEstimatorSession session(process_noise);
    session.initialize(10.0, x, Eigen::MatrixXd::Identity(StateIndex::dimension, StateIndex::dimension));
    const auto update = session.updateRtk(10.0, rtk_position, rtk_velocity,
                                          0.01 * Eigen::Matrix<double, 6, 6>::Identity());
    if (!update.accepted || (session.state() - x).norm() <= 0.0)
      throw std::runtime_error("GMIS LC RTK update did not change/accept state");
    if (!session.covariance().allFinite()) throw std::runtime_error("GMIS LC covariance is not finite");
    std::cout << "gmis_lc_math_validation=PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "gmis_lc_math_validation=FAIL: " << e.what() << '\n';
    return 1;
  }
}
