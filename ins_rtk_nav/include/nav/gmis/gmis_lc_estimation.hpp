#pragma once

#include "nav/gmis/gmis_lc_models.hpp"
#include "nav/gmis/gmis_lc_config.hpp"
#include "navcore/state_estimator.hpp"

#include <optional>

namespace nav::gmis {

// GMIS full-state session. It is intentionally not an IModule and has no
// dependency on the TMIS INS mechanization or error-feedback transaction.
class GmisLcEstimatorSession {
public:
  explicit GmisLcEstimatorSession(GmisProcessNoise process_noise = {});
  explicit GmisLcEstimatorSession(GmisLcConfiguration configuration);

  void initialize(double time_seconds, const Eigen::VectorXd& state,
                  const Eigen::MatrixXd& covariance);
  nav::core::EstimatorUpdateResult updateImu(
      const ImuSample& sample,
      const Eigen::Matrix<double, 6, 6>& measurement_covariance);
  nav::core::EstimatorUpdateResult updateImu(const ImuSample& sample);
  nav::core::EstimatorUpdateResult updateRtk(
      double time_seconds, const Eigen::Vector3d& position_ned_m,
      const Eigen::Vector3d& velocity_ned_mps,
      const Eigen::Matrix<double, 6, 6>& measurement_covariance,
      const Eigen::Vector3d& lever_arm_body_m = Eigen::Vector3d::Zero());
  nav::core::EstimatorUpdateResult updateRtk(
      double time_seconds, const Eigen::Vector3d& position_ned_m,
      const Eigen::Vector3d& velocity_ned_mps);

  const Eigen::VectorXd& state() const noexcept { return estimator_.state(); }
  const Eigen::MatrixXd& covariance() const noexcept { return estimator_.covariance(); }
  double timeSeconds() const noexcept { return time_seconds_; }
  bool initialized() const noexcept { return initialized_; }

private:
  void predictTo(double time_seconds);

  GmisProcessNoise process_noise_;
  std::optional<GmisLcConfiguration> configuration_;
  nav::core::StateEstimator estimator_;
  double time_seconds_ = 0.0;
  bool initialized_ = false;
};

} // namespace nav::gmis
