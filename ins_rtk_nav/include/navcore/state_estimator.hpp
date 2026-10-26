#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>

namespace navcore {

// Domain-agnostic linear state/covariance estimator. It has no navigation,
// sensor, or measurement-model dependencies; callers supply all matrices.
struct EstimatorUpdateOptions {
  std::optional<double> nis_gate;
  double minimum_variance = 1e-12;
};

struct EstimatorUpdateResult {
  bool accepted = false;
  std::string reason;
  Eigen::VectorXd innovation;
  Eigen::VectorXd normalized_innovation;
  Eigen::MatrixXd innovation_covariance;
  double nis = 0.0;
};

class StateEstimator {
public:
  void initialize(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance);
  void predict(const Eigen::MatrixXd& transition, const Eigen::MatrixXd& process_noise);
  EstimatorUpdateResult update(const Eigen::VectorXd& innovation,
                              const Eigen::MatrixXd& observation_jacobian,
                              const Eigen::MatrixXd& measurement_noise,
                              const EstimatorUpdateOptions& options = {});
  void resetErrorState(const Eigen::MatrixXd& reset_jacobian);
  const Eigen::VectorXd& state() const noexcept { return state_; }
  const Eigen::MatrixXd& covariance() const noexcept { return covariance_; }
  bool initialized() const noexcept { return initialized_; }

private:
  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
  bool initialized_ = false;
};

} // namespace navcore
