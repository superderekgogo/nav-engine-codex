#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <deque>
#include <map>
#include <set>
#include <string>

namespace navcore { class Json; }

namespace nav::gmis::vce {

struct VceConfig {
  bool enabled = false;
  std::string mode = "diagnostic_only";
  bool apply_factors = false;
  double window_seconds = 30.0;
  std::size_t minimum_valid_epochs = 20;
  double minimum_total_redundancy = 5.0;
  double smoothing_factor = 0.15;
  std::set<std::string> component_ids;

  static VceConfig fromJson(const navcore::Json& json);
};

struct VceComponentDiagnostic {
  bool enabled = false;
  bool sample_valid = false;
  bool window_valid = false;
  std::string component_id;
  std::string status;
  double quadratic_form = 0.0;
  double redundancy = 0.0;
  double epoch_variance_factor = 0.0;
  double window_quadratic_form = 0.0;
  double window_redundancy = 0.0;
  double window_variance_factor = 0.0;
  double smoothed_variance_factor = 0.0;
  std::size_t valid_epochs = 0;
};

// Diagnostic-only online Helmert VCE accumulator. It consumes post-fit
// residuals from accepted filter updates and never modifies navigation state,
// process noise, or measurement covariance.
class VceEngine {
public:
  explicit VceEngine(VceConfig config);

  VceComponentDiagnostic observe(double time_seconds,
                                 const std::string& component_id,
                                 const Eigen::VectorXd& posterior_residual,
                                 const Eigen::MatrixXd& cofactor,
                                 const Eigen::MatrixXd& posterior_residual_covariance,
                                 bool update_accepted);

  const VceConfig& config() const noexcept { return config_; }

private:
  struct Sample {
    double time_seconds = 0.0;
    double quadratic_form = 0.0;
    double redundancy = 0.0;
  };
  struct ComponentWindow {
    std::deque<Sample> samples;
    double smoothed_factor = 0.0;
    bool has_smoothed_factor = false;
  };

  VceConfig config_;
  std::map<std::string, ComponentWindow> windows_;
};

} // namespace nav::gmis::vce
