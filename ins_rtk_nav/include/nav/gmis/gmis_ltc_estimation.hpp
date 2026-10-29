#pragma once

#include "nav/gmis/gmis_lc_config.hpp"
#include "navcore/state_estimator.hpp"
#include "navcore/messages.hpp"

namespace nav::gmis {

// GMIS-LTC's temporary 42D joint state is [x(previous GNSS), x(current)].
// The old endpoint is frozen while the current endpoint receives IMU updates.
class GmisLtcEstimatorSession {
public:
  explicit GmisLtcEstimatorSession(GmisLcConfiguration configuration,
                                   double clone_regularization = 1e-9);
  void initialize(double time, const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance);
  nav::core::EstimatorUpdateResult updateImu(const ImuSample& sample);
  nav::core::EstimatorUpdateResult updateEpoch(
      double time, const Eigen::Vector3d& antenna_position_ned,
      const Eigen::Matrix3d& position_covariance,
      const Eigen::Vector3d& base_position_ecef,
      const navcore::PreparedTdDdCarrierPhaseEpoch* tdcp,
      double tdcp_variance_inflation);
  nav::core::EstimatorUpdateResult updateZupt(double sigma_mps, double nis_gate);
  nav::core::EstimatorUpdateResult updateNhc(double sigma_lateral_mps, double sigma_vertical_mps,
                                              double nis_gate);
  nav::core::EstimatorUpdateResult updateZeroPositionChange(double sigma_m, double nis_gate);
  nav::core::EstimatorUpdateResult updateZeroHeadingChange(double sigma_rad, double nis_gate);
  void rollLagWindow();
  const Eigen::VectorXd& state() const noexcept { return estimator_.state(); }
  const Eigen::MatrixXd& covariance() const noexcept { return estimator_.covariance(); }
  double timeSeconds() const noexcept { return time_seconds_; }
  std::size_t tdcpRows() const noexcept { return tdcp_rows_; }
  double lastTdcpNis() const noexcept { return tdcp_nis_; }
private:
  Eigen::Vector3d antennaEcef(const Eigen::VectorXd& state) const;
  double ddGeometry(const Eigen::Vector3d& receiver,
                    const navcore::TdDdCarrierPhaseObservation& row, bool current,
                    const Eigen::Vector3d& base_ecef) const;
  GmisLcConfiguration config_;
  nav::core::StateEstimator estimator_;
  double time_seconds_ = 0.0;
  double clone_regularization_ = 1e-9;
  bool initialized_ = false;
  std::size_t tdcp_rows_ = 0;
  double tdcp_nis_ = 0.0;
};

} // namespace nav::gmis
