#pragma once

#include "nav/gmis/gmis_lc_models.hpp"
#include "navcore/json.hpp"

#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace nav::gmis::ltc {

struct DynamicStatus {
  double time_seconds = 0.0;
  std::string state = "unknown";
  bool high_dynamic = false;
  double confidence = 0.0;
  double speed_mps = 0.0;
  double gyro_rms_radps = 0.0;
  double specific_force_norm_std_mps2 = 0.0;
};

struct DynamicStatusConfig {
  double window_seconds = 0.5;
  double minimum_dwell_seconds = 0.8;
  double stationary_gyro_rms_radps = 0.02;
  double stationary_force_norm_std_mps2 = 0.08;
  double stationary_speed_mps = 0.3;
  double moving_gyro_rms_radps = 0.035;
  double moving_force_norm_std_mps2 = 0.12;
  double moving_speed_mps = 0.6;
  double high_dynamic_gyro_rms_radps = 0.2;
  double high_dynamic_force_norm_std_mps2 = 0.3;
  static DynamicStatusConfig fromJson(const navcore::Json& json);
};

struct ConstraintConfig {
  bool use_zupt = false;
  bool use_nhc = false;
  bool use_zpc = false;
  bool use_zhc = false;
  double minimum_confidence = 0.5;
  double minimum_update_interval_seconds = 1.0;
  double zupt_sigma_mps = 0.05;
  double nhc_sigma_lateral_mps = 0.1;
  double nhc_sigma_vertical_mps = 0.1;
  double zpc_sigma_m = 0.05;
  double zhc_sigma_heading_rad = 0.5 * 3.14159265358979323846 / 180.0;
  double nis_gate = 16.27;
  static ConstraintConfig fromJson(const navcore::Json& json);
};

class DynamicStatusModule {
public:
  explicit DynamicStatusModule(DynamicStatusConfig config);
  void push(const ImuSample& sample, const Eigen::Vector3d& gyro_bias,
            const Eigen::Vector3d& accel_bias);
  DynamicStatus evaluate(double time_seconds, double predicted_speed_mps,
                         std::optional<double> rtk_speed_mps = std::nullopt);
  void reset();
private:
  struct Sample { double time = 0.0; double gyro_norm = 0.0; double force_norm = 0.0; };
  DynamicStatusConfig config_;
  std::deque<Sample> window_;
  std::string state_ = "unknown", candidate_;
  double candidate_since_ = 0.0;
};

class ConstraintManagerModule {
public:
  explicit ConstraintManagerModule(ConstraintConfig config);
  std::vector<std::string> select(const DynamicStatus& status);
  void reset();
private:
  ConstraintConfig config_;
  double last_published_seconds_ = -1.0;
};

} // namespace nav::gmis::ltc
