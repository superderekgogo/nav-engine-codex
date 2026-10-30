#include "nav/gmis/gmis_vce.hpp"
#include "navcore/json.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace nav::gmis::vce {
namespace {

double number(const navcore::Json& object, const char* key, double fallback) {
  const auto* value = object.find(key);
  if (value == nullptr) return fallback;
  if (!value->isNumber()) throw std::invalid_argument(std::string("VCE config field must be numeric: ") + key);
  const double result = value->asNumber();
  if (!std::isfinite(result)) throw std::invalid_argument(std::string("VCE config field must be finite: ") + key);
  return result;
}

bool boolean(const navcore::Json& object, const char* key, bool fallback) {
  const auto* value = object.find(key);
  if (value == nullptr) return fallback;
  if (!value->isBool()) throw std::invalid_argument(std::string("VCE config field must be boolean: ") + key);
  return value->asBool();
}

} // namespace

VceConfig VceConfig::fromJson(const navcore::Json& json) {
  if (!json.isObject()) throw std::invalid_argument("VCE configuration must be an object");
  VceConfig config;
  config.enabled = boolean(json, "enabled", false);
  if (const auto* mode = json.find("mode")) {
    if (!mode->isString()) throw std::invalid_argument("VCE mode must be a string");
    config.mode = mode->asString();
  }
  if (config.mode != "diagnostic_only")
    throw std::invalid_argument("only diagnostic_only VCE mode is currently implemented");
  config.apply_factors = boolean(json, "apply_factors", false);
  if (config.apply_factors)
    throw std::invalid_argument("VCE covariance feedback is not implemented; apply_factors must be false");

  if (const auto* window = json.find("window")) {
    if (!window->isObject()) throw std::invalid_argument("VCE window must be an object");
    config.window_seconds = number(*window, "duration_seconds", config.window_seconds);
    const double minimum_epochs = number(*window, "minimum_valid_epochs",
                                         static_cast<double>(config.minimum_valid_epochs));
    if (minimum_epochs < 1.0 || std::floor(minimum_epochs) != minimum_epochs)
      throw std::invalid_argument("VCE minimum_valid_epochs must be a positive integer");
    config.minimum_valid_epochs = static_cast<std::size_t>(minimum_epochs);
    config.minimum_total_redundancy = number(*window, "minimum_total_redundancy",
                                              config.minimum_total_redundancy);
    config.smoothing_factor = number(*window, "smoothing_factor", config.smoothing_factor);
  }
  if (!(config.window_seconds > 0.0) || !(config.minimum_total_redundancy >= 0.0) ||
      !(config.smoothing_factor > 0.0 && config.smoothing_factor <= 1.0))
    throw std::invalid_argument("invalid VCE window, redundancy threshold, or smoothing factor");

  if (const auto* components = json.find("components")) {
    if (!components->isArray()) throw std::invalid_argument("VCE components must be an array");
    for (const auto& item : components->asArray()) {
      if (!item.isString() || item.asString().empty())
        throw std::invalid_argument("VCE component IDs must be non-empty strings");
      if (!config.component_ids.insert(item.asString()).second)
        throw std::invalid_argument("duplicate VCE component ID: " + item.asString());
    }
  }
  if (config.enabled && config.component_ids.empty())
    throw std::invalid_argument("enabled VCE needs at least one component ID");
  return config;
}

VceEngine::VceEngine(VceConfig config) : config_(std::move(config)) {}

VceComponentDiagnostic VceEngine::observe(
    double time_seconds, const std::string& component_id,
    const Eigen::VectorXd& residual, const Eigen::MatrixXd& cofactor,
    const Eigen::MatrixXd& residual_covariance, bool update_accepted) {
  VceComponentDiagnostic result;
  result.enabled = config_.enabled;
  result.component_id = component_id;
  result.status = "disabled";
  if (!config_.enabled) return result;
  if (config_.component_ids.find(component_id) == config_.component_ids.end()) {
    result.status = "component_not_configured";
    return result;
  }
  result.status = "update_rejected";
  if (!update_accepted) return result;
  if (!std::isfinite(time_seconds)) {
    result.status = "invalid_timestamp";
    return result;
  }
  const Eigen::Index m = residual.size();
  if (m == 0 || cofactor.rows() != m || cofactor.cols() != m ||
      residual_covariance.rows() != m || residual_covariance.cols() != m) {
    result.status = "dimension_mismatch";
    return result;
  }
  if (!residual.allFinite() || !cofactor.allFinite() || !residual_covariance.allFinite()) {
    result.status = "non_finite_input";
    return result;
  }

  const Eigen::MatrixXd d = 0.5 * (cofactor + cofactor.transpose());
  Eigen::LDLT<Eigen::MatrixXd> factor(d);
  if (factor.info() != Eigen::Success || !factor.isPositive()) {
    result.status = "cofactor_not_positive_definite";
    return result;
  }
  const Eigen::VectorXd whitened_residual = factor.solve(residual);
  const Eigen::MatrixXd whitened_covariance = factor.solve(
      0.5 * (residual_covariance + residual_covariance.transpose()));
  if (factor.info() != Eigen::Success || !whitened_residual.allFinite() ||
      !whitened_covariance.allFinite()) {
    result.status = "cofactor_solve_failed";
    return result;
  }
  const double q = residual.dot(whitened_residual);
  const double redundancy = whitened_covariance.trace();
  if (!std::isfinite(q) || !std::isfinite(redundancy) || q < -1e-10 || redundancy <= 1e-10) {
    result.status = "insufficient_or_invalid_redundancy";
    return result;
  }

  result.sample_valid = true;
  result.status = "accumulating";
  result.quadratic_form = std::max(0.0, q);
  result.redundancy = redundancy;
  result.epoch_variance_factor = result.quadratic_form / redundancy;
  auto& window = windows_[component_id];
  if (!window.samples.empty() && time_seconds < window.samples.back().time_seconds) {
    result.sample_valid = false;
    result.status = "out_of_order_sample";
    return result;
  }
  window.samples.push_back({time_seconds, result.quadratic_form, redundancy});
  const double oldest_time = time_seconds - config_.window_seconds;
  while (!window.samples.empty() && window.samples.front().time_seconds < oldest_time)
    window.samples.pop_front();

  for (const auto& sample : window.samples) {
    result.window_quadratic_form += sample.quadratic_form;
    result.window_redundancy += sample.redundancy;
  }
  result.valid_epochs = window.samples.size();
  if (result.valid_epochs < config_.minimum_valid_epochs ||
      result.window_redundancy < config_.minimum_total_redundancy) {
    result.status = "insufficient_window_redundancy";
    return result;
  }
  result.window_valid = true;
  result.window_variance_factor = result.window_quadratic_form / result.window_redundancy;
  const double factor_for_smoothing = std::max(result.window_variance_factor, 1e-12);
  if (!window.has_smoothed_factor) {
    window.smoothed_factor = factor_for_smoothing;
    window.has_smoothed_factor = true;
  } else {
    const double alpha = config_.smoothing_factor;
    window.smoothed_factor = std::exp(alpha * std::log(factor_for_smoothing) +
                                      (1.0 - alpha) * std::log(window.smoothed_factor));
  }
  result.smoothed_variance_factor = window.smoothed_factor;
  result.status = "valid_diagnostic_only";
  return result;
}

} // namespace nav::gmis::vce
