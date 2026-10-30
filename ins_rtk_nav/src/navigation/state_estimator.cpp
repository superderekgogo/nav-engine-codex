#include "navcore/state_estimator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>

namespace navcore {
namespace {
void requireFinite(const Eigen::MatrixXd& matrix, const char* name) {
  if (!matrix.allFinite()) throw std::invalid_argument(std::string(name) + " contains non-finite values");
}
}

void StateEstimator::initialize(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance) {
  if (state.size() == 0 || covariance.rows() != state.size() || covariance.cols() != state.size())
    throw std::invalid_argument("StateEstimator initialize dimension mismatch");
  requireFinite(state, "state");
  requireFinite(covariance, "covariance");
  Eigen::MatrixXd symmetric = 0.5 * (covariance + covariance.transpose());
  Eigen::LDLT<Eigen::MatrixXd> factor(symmetric);
  if (factor.info() != Eigen::Success || !factor.isPositive())
    throw std::invalid_argument("StateEstimator covariance must be positive definite");
  state_ = state;
  covariance_ = std::move(symmetric);
  initialized_ = true;
}

void StateEstimator::predict(const Eigen::MatrixXd& transition, const Eigen::MatrixXd& process_noise) {
  if (!initialized_) throw std::logic_error("StateEstimator is not initialized");
  const Eigen::Index n = state_.size();
  if (transition.rows() != n || transition.cols() != n || process_noise.rows() != n || process_noise.cols() != n)
    throw std::invalid_argument("StateEstimator prediction dimension mismatch");
  requireFinite(transition, "transition");
  requireFinite(process_noise, "process noise");
  state_ = transition * state_;
  covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  requireFinite(state_, "predicted state");
  requireFinite(covariance_, "predicted covariance");
}

void StateEstimator::predict(const Eigen::VectorXd& predicted_state,
                             const Eigen::MatrixXd& transition_jacobian,
                             const Eigen::MatrixXd& process_noise) {
  if (!initialized_) throw std::logic_error("StateEstimator is not initialized");
  const Eigen::Index n = state_.size();
  if (predicted_state.size() != n || transition_jacobian.rows() != n ||
      transition_jacobian.cols() != n || process_noise.rows() != n || process_noise.cols() != n)
    throw std::invalid_argument("StateEstimator nonlinear prediction dimension mismatch");
  requireFinite(predicted_state, "predicted state");
  requireFinite(transition_jacobian, "transition Jacobian");
  requireFinite(process_noise, "process noise");
  state_ = predicted_state;
  covariance_ = transition_jacobian * covariance_ * transition_jacobian.transpose() + process_noise;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  requireFinite(covariance_, "predicted covariance");
}

EstimatorUpdateResult StateEstimator::update(const Eigen::VectorXd& innovation,
                                             const Eigen::MatrixXd& observation_jacobian,
                                             const Eigen::MatrixXd& measurement_noise,
                                             const EstimatorUpdateOptions& options) {
  if (!initialized_) throw std::logic_error("StateEstimator is not initialized");
  const Eigen::Index n = state_.size(), m = innovation.size();
  if (m == 0 || observation_jacobian.rows() != m || observation_jacobian.cols() != n ||
      measurement_noise.rows() != m || measurement_noise.cols() != m)
    throw std::invalid_argument("StateEstimator update dimension mismatch");
  requireFinite(innovation, "innovation");
  requireFinite(observation_jacobian, "observation Jacobian");
  requireFinite(measurement_noise, "measurement noise");
  if (!(options.minimum_variance > 0.0) || !std::isfinite(options.minimum_variance))
    throw std::invalid_argument("minimum_variance must be finite and positive");

  Eigen::MatrixXd r = 0.5 * (measurement_noise + measurement_noise.transpose());
  r.diagonal() = r.diagonal().cwiseMax(options.minimum_variance);
  Eigen::MatrixXd s = observation_jacobian * covariance_ * observation_jacobian.transpose() + r;
  s = 0.5 * (s + s.transpose());
  Eigen::LDLT<Eigen::MatrixXd> factor(s);
  if (factor.info() != Eigen::Success || !factor.isPositive()) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(s, Eigen::EigenvaluesOnly);
    const double minimum = eigen.info() == Eigen::Success ? eigen.eigenvalues().minCoeff() : -1.0;
    throw std::runtime_error("StateEstimator innovation covariance is not positive definite (minimum eigenvalue=" + std::to_string(minimum) + ")");
  }
  const Eigen::VectorXd solved = factor.solve(innovation);
  if (factor.info() != Eigen::Success || !solved.allFinite())
    throw std::runtime_error("StateEstimator innovation solve failed");

  EstimatorUpdateResult result;
  result.innovation = innovation;
  result.innovation_covariance = s;
  result.normalized_innovation = innovation.cwiseQuotient(s.diagonal().cwiseSqrt());
  result.nis = innovation.dot(solved);
  if (options.nis_gate && result.nis > *options.nis_gate) {
    result.reason = "NIS gate rejected measurement";
    return result;
  }

  const Eigen::MatrixXd gain = covariance_ * observation_jacobian.transpose() * factor.solve(Eigen::MatrixXd::Identity(m, m));
  const Eigen::VectorXd posterior_residual = observation_jacobian * (gain * innovation) - innovation;
  state_ += gain * innovation;
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n, n);
  const Eigen::MatrixXd ikh = identity - gain * observation_jacobian;
  covariance_ = ikh * covariance_ * ikh.transpose() + gain * r * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  requireFinite(state_, "updated state");
  requireFinite(covariance_, "updated covariance");
  result.accepted = true;
  result.posterior_residual = posterior_residual;
  result.posterior_residual_covariance = r - observation_jacobian * covariance_ * observation_jacobian.transpose();
  result.posterior_residual_covariance = 0.5 * (result.posterior_residual_covariance +
                                                 result.posterior_residual_covariance.transpose());
  requireFinite(result.posterior_residual, "posterior observation residual");
  requireFinite(result.posterior_residual_covariance, "posterior residual covariance");
  return result;
}

void StateEstimator::resetErrorState(const Eigen::MatrixXd& reset_jacobian) {
  if (!initialized_) throw std::logic_error("StateEstimator is not initialized");
  const Eigen::Index n = state_.size();
  if (reset_jacobian.rows() != n || reset_jacobian.cols() != n)
    throw std::invalid_argument("StateEstimator reset Jacobian dimension mismatch");
  requireFinite(reset_jacobian, "reset Jacobian");
  covariance_ = reset_jacobian * covariance_ * reset_jacobian.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  state_.setZero();
  requireFinite(covariance_, "reset covariance");
}

} // namespace navcore
