#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <navcore/json.hpp>

namespace nav::gmis {

// Phase-one GMIS state convention: NED navigation frame, FRD body frame,
// Euler [roll, pitch, heading] in radians; cNedToBody maps NED vectors to body.
struct StateIndex {
  static constexpr int position = 0;
  static constexpr int velocity = 3;
  static constexpr int acceleration = 6;
  static constexpr int attitude = 9;
  static constexpr int angular_rate = 12;
  static constexpr int gyro_bias = 15;
  static constexpr int accel_bias = 18;
  static constexpr int dimension = 21;
};

struct ImuSample {
  double time_seconds = 0.0;
  enum class GyroRateConvention {
    Unspecified,
    BodyRelativeNavigation,
    BodyRelativeInertial
  } gyro_rate_convention = GyroRateConvention::Unspecified;
  Eigen::Vector3d gyro_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d specific_force_mps2 = Eigen::Vector3d::Zero();
};

struct GeodeticOrigin {
  double latitude_rad = 0.0;
  double longitude_rad = 0.0;
  double height_m = 0.0;
};

struct EarthNavigationTerms {
  Eigen::Vector3d earth_rate_n_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d transport_rate_n_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_n_mps2 = Eigen::Vector3d(0.0, 0.0, 9.80665);
};

struct GmisProcessNoise {
  // Discrete per-axis variances for constant jerk and angular acceleration.
  Eigen::Vector3d jerk_variance = Eigen::Vector3d::Constant(1.0);
  Eigen::Vector3d angular_acceleration_variance = Eigen::Vector3d::Constant(1e-4);
  // Continuous random-walk power spectral densities (state_unit^2 / second).
  Eigen::Vector3d gyro_bias_rw_psd = Eigen::Vector3d::Constant(1e-10);
  Eigen::Vector3d accel_bias_rw_psd = Eigen::Vector3d::Constant(1e-8);
};

struct Prediction {
  Eigen::VectorXd state;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd process_noise;
};

struct LinearizedObservation {
  Eigen::VectorXd innovation; // observed - predicted
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd noise;
};

class GmisLcModels {
public:
  static Eigen::Matrix3d cNedToBody(const Eigen::Vector3d& roll_pitch_heading_rad);
  static std::array<Eigen::Matrix3d, 3> cNedToBodyPartials(
      const Eigen::Vector3d& roll_pitch_heading_rad);
  static Eigen::Matrix3d eulerRateMatrix(double pitch_rad, double roll_rad);
  static EarthNavigationTerms earthNavigationTerms(
      const Eigen::Vector3d& position_ned_m,
      const Eigen::Vector3d& velocity_ned_mps,
      const GeodeticOrigin& origin);
  static Eigen::Matrix<double, 6, 1> predictEarthAwareImu(
      const Eigen::VectorXd& state, const ImuSample& sample,
      const GeodeticOrigin& origin);

  static Prediction predict(const Eigen::VectorXd& state, double dt_seconds,
                            const GmisProcessNoise& noise);
  static LinearizedObservation linearizeImu(
      const Eigen::VectorXd& state, const ImuSample& sample,
      const Eigen::Matrix<double, 6, 6>& measurement_covariance,
      const Eigen::Vector3d& gravity_ned_mps2 = {0.0, 0.0, 9.80665});
  static LinearizedObservation linearizeImuEarthAware(
      const Eigen::VectorXd& state, const ImuSample& sample,
      const Eigen::Matrix<double, 6, 6>& measurement_covariance,
      const GeodeticOrigin& origin);
  static LinearizedObservation linearizeRtkPositionVelocity(
      const Eigen::VectorXd& state, const Eigen::Vector3d& position_ned_m,
      const Eigen::Vector3d& velocity_ned_mps,
      const Eigen::Matrix<double, 6, 6>& measurement_covariance,
      const Eigen::Vector3d& lever_arm_body_m = Eigen::Vector3d::Zero());

  static Eigen::Vector3d normalizeEuler(const Eigen::Vector3d& angles_rad);
};

} // namespace nav::gmis
