#include "nav/gmis/gmis_lc_estimation.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace nav::gmis {

GmisLcEstimatorSession::GmisLcEstimatorSession(GmisProcessNoise process_noise)
    : process_noise_(std::move(process_noise)) {}

GmisLcEstimatorSession::GmisLcEstimatorSession(GmisLcConfiguration configuration)
    : process_noise_(configuration.process_noise), configuration_(std::move(configuration)) {}

void GmisLcEstimatorSession::initialize(double time, const Eigen::VectorXd& state,
                                        const Eigen::MatrixXd& covariance) {
  if (!std::isfinite(time) || state.size() != StateIndex::dimension)
    throw std::invalid_argument("invalid GMIS initialization time or state dimension");
  estimator_.initialize(state, covariance);
  time_seconds_ = time;
  initialized_ = true;
}

void GmisLcEstimatorSession::predictTo(double time) {
  if (!initialized_) throw std::logic_error("GMIS estimator is not initialized");
  if (!std::isfinite(time) || time < time_seconds_)
    throw std::invalid_argument("GMIS events must be finite and ordered by event time");
  if (time == time_seconds_) return;
  const Prediction prediction = GmisLcModels::predict(estimator_.state(), time - time_seconds_, process_noise_);
  estimator_.predict(prediction.state, prediction.jacobian, prediction.process_noise);
  time_seconds_ = time;
}

nav::core::EstimatorUpdateResult GmisLcEstimatorSession::updateImu(
    const ImuSample& sample,
    const Eigen::Matrix<double, 6, 6>& measurement_covariance) {
  predictTo(sample.time_seconds);
  const auto observation = configuration_ &&
                                   configuration_->imu_model == GmisImuModelKind::EarthAwareInertialRate
      ? GmisLcModels::linearizeImuEarthAware(estimator_.state(), sample, measurement_covariance,
                                             configuration_->origin)
      : GmisLcModels::linearizeImu(estimator_.state(), sample, measurement_covariance);
  return estimator_.update(observation.innovation, observation.jacobian, observation.noise);
}

nav::core::EstimatorUpdateResult GmisLcEstimatorSession::updateImu(const ImuSample& sample) {
  if (!configuration_) throw std::logic_error("GMIS IMU update needs a loaded model configuration");
  return updateImu(sample, configuration_->imu_measurement_covariance);
}

nav::core::EstimatorUpdateResult GmisLcEstimatorSession::updateRtk(
    double time, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    const Eigen::Matrix<double, 6, 6>& measurement_covariance,
    const Eigen::Vector3d& lever_arm_body_m) {
  predictTo(time);
  const auto observation = GmisLcModels::linearizeRtkPositionVelocity(
      estimator_.state(), position, velocity, measurement_covariance,
      configuration_ ? configuration_->antenna_lever_arm_body_m : lever_arm_body_m);
  return estimator_.update(observation.innovation, observation.jacobian, observation.noise);
}

nav::core::EstimatorUpdateResult GmisLcEstimatorSession::updateRtk(
    double time, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
  if (!configuration_) throw std::logic_error("GMIS RTK update needs a loaded model configuration");
  return updateRtk(time, position, velocity, configuration_->rtk_measurement_covariance);
}

} // namespace nav::gmis
