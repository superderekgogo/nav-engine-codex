#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>

namespace navcore {
namespace {
class DynamicStatusModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "dynamic_status"; }
  Status configure(const Json& cfg) override {
    inputTopic_ = cfg.at("subscribe_topic").asString();
    outputTopic_ = cfg.at("publish_topic").asString();
    const auto& c = cfg.at("fusion_model").at("dynamic_status");
    windowSeconds_ = c.at("window_seconds").asNumber();
    minimumDwellSeconds_ = c.at("minimum_dwell_seconds").asNumber();
    stationaryGyroRms_ = c.at("stationary_gyro_rms_radps").asNumber();
    stationaryForceStd_ = c.at("stationary_force_norm_std_mps2").asNumber();
    stationarySpeed_ = c.at("stationary_speed_mps").asNumber();
    movingGyroRms_ = c.at("moving_gyro_rms_radps").asNumber();
    movingForceStd_ = c.at("moving_force_norm_std_mps2").asNumber();
    movingSpeed_ = c.at("moving_speed_mps").asNumber();
    highDynamicGyroRms_ = c.at("high_dynamic_gyro_rms_radps").asNumber();
    highDynamicForceStd_ = c.at("high_dynamic_force_norm_std_mps2").asNumber();
    if (windowSeconds_ <= 0 || minimumDwellSeconds_ < 0 || stationaryGyroRms_ <= 0 || stationaryForceStd_ <= 0 ||
        stationarySpeed_ < 0 || movingGyroRms_ <= stationaryGyroRms_ || movingForceStd_ <= stationaryForceStd_ ||
        movingSpeed_ <= stationarySpeed_ || highDynamicGyroRms_ <= movingGyroRms_ || highDynamicForceStd_ <= movingForceStd_)
      return Status::failure("invalid dynamic_status thresholds");
    return Status::success();
  }
  Status initialize(ModuleContext& ctx) override {
    bus_ = &ctx.bus;
    ctx.bus.subscribe<ReplayControl>("navigation.replay.control", [this](const ReplayControl& r) { if (r.reset) reset(); });
    ctx.bus.subscribe<RtkPositionVelocityObservation>("navigation.rtk", [this](const auto& rtk) { rtk_.push_back(rtk); });
    ctx.bus.subscribe<InertialStateSnapshot>(inputTopic_, [this](const auto& snapshot) { onSnapshot(snapshot); });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override { window_.clear(); }
private:
  void reset() { window_.clear(); state_ = "unknown"; candidate_.clear(); candidateSince_ = 0.0; }
  void onSnapshot(const InertialStateSnapshot& s) {
    window_.push_back(s);
    while (window_.size() > 2 && s.gps_seconds - window_.front().gps_seconds > windowSeconds_) window_.pop_front();
    DynamicStatus out;
    out.gps_seconds = s.gps_seconds;
    out.epoch_id = s.epoch_id;
    out.speed_mps = s.predicted_state.velocity_ned_mps.norm();
    bool rtkSpeedValid = false;
    const auto match = std::lower_bound(rtk_.begin(), rtk_.end(), s.gps_seconds,
      [](const auto& value, double t) { return value.gps_seconds < t; });
    const RtkPositionVelocityObservation* rtk = nullptr;
    if (match != rtk_.end()) rtk = &*match;
    if (match != rtk_.begin() && (!rtk || std::abs((match - 1)->gps_seconds - s.gps_seconds) < std::abs(rtk->gps_seconds - s.gps_seconds))) rtk = &*(match - 1);
    if (rtk && std::abs(rtk->gps_seconds - s.gps_seconds) <= 0.51 && (rtk->valid_components & 0x38u) == 0x38u &&
        rtk->covariance.block<3, 3>(3, 3).trace() < 12.0) {
      const double lat = s.predicted_state.lla_rad_m.x(), lon = s.predicted_state.lla_rad_m.y();
      Eigen::Matrix3d ecefToNed;
      ecefToNed << -std::sin(lat)*std::cos(lon), -std::sin(lat)*std::sin(lon), std::cos(lat),
                   -std::sin(lon), std::cos(lon), 0.0,
                   -std::cos(lat)*std::cos(lon), -std::cos(lat)*std::sin(lon), -std::sin(lat);
      out.speed_mps = (ecefToNed * rtk->antenna_velocity_ecef_mps).norm();
      rtkSpeedValid = true;
    }
    if (window_.size() < 5 || s.gps_seconds - window_.front().gps_seconds < 0.8 * windowSeconds_) {
      bus_->publish(outputTopic_, out);
      return;
    }
    double gyro2 = 0.0, forceMean = 0.0, force2 = 0.0;
    for (const auto& sample : window_) {
      const auto& x = sample.imu;
      const Eigen::Vector3d omega = x.gyro_radps - sample.predicted_state.gyro_bias_radps;
      const double g = omega.norm();
      const double f = (x.specific_force_mps2 - sample.predicted_state.accel_bias_mps2).norm();
      gyro2 += g * g;
      forceMean += f;
      force2 += f * f;
    }
    const double n = static_cast<double>(window_.size());
    out.gyro_rms_radps = std::sqrt(gyro2 / n);
    forceMean /= n;
    out.specific_force_norm_std_mps2 = std::sqrt(std::max(0.0, force2 / n - forceMean * forceMean));
    const bool stationary = out.gyro_rms_radps < stationaryGyroRms_ &&
                            out.specific_force_norm_std_mps2 < stationaryForceStd_ && (!rtkSpeedValid || out.speed_mps < stationarySpeed_);
    const bool moving = out.gyro_rms_radps > movingGyroRms_ ||
                        out.specific_force_norm_std_mps2 > movingForceStd_ || (rtkSpeedValid && out.speed_mps > movingSpeed_);
    out.high_dynamic = out.gyro_rms_radps > highDynamicGyroRms_ || out.specific_force_norm_std_mps2 > highDynamicForceStd_;
    std::string proposed = stationary ? "stationary" : (moving ? "moving" : "unknown");
    if (proposed == state_) { candidate_.clear(); candidateSince_ = 0.0; }
    else if (proposed != "unknown") {
      if (candidate_ != proposed) { candidate_ = proposed; candidateSince_ = s.gps_seconds; }
      if (s.gps_seconds - candidateSince_ >= minimumDwellSeconds_) { state_ = proposed; candidate_.clear(); }
    } else { candidate_.clear(); candidateSince_ = 0.0; }
    out.state = state_;
    out.confidence = state_ == "stationary" ? 0.95 : (state_ == "moving" ? 0.75 : 0.0);
    bus_->publish(outputTopic_, out);
  }
  MessageBus* bus_ = nullptr;
  std::string inputTopic_, outputTopic_, state_ = "unknown", candidate_;
  std::deque<InertialStateSnapshot> window_;
  std::vector<RtkPositionVelocityObservation> rtk_;
  double windowSeconds_ = 0.5, minimumDwellSeconds_ = 0.8, stationaryGyroRms_ = 0.02,
         stationaryForceStd_ = 0.08, stationarySpeed_ = 0.3, movingGyroRms_ = 0.035,
         movingForceStd_ = 0.12, movingSpeed_ = 0.6, highDynamicGyroRms_ = 0.2,
         highDynamicForceStd_ = 0.3, candidateSince_ = 0.0;
};

class ConstraintManagerModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "constraint_manager"; }
  Status configure(const Json& cfg) override {
    inputTopic_ = cfg.at("subscribe_topic").asString();
    outputTopic_ = cfg.at("publish_topic").asString();
    const auto& fusion = cfg.at("fusion_model");
    for (const auto& item : fusion.at("measurement_models").asArray()) {
      const auto& id = item.asString();
      zuptEnabled_ = zuptEnabled_ || id == "zupt_stationary_v1";
      nhcEnabled_ = nhcEnabled_ || id == "vehicle_nhc_v1";
      zpcEnabled_ = zpcEnabled_ || id == "zero_position_change_v1";
      zhcEnabled_ = zhcEnabled_ || id == "zero_heading_change_v1";
    }
    const auto& c = fusion.at("constraints");
    minimumConfidence_ = c.at("minimum_confidence").asNumber();
    minIntervalSeconds_ = c.at("minimum_update_interval_seconds").asNumber();
    if (minimumConfidence_ < 0 || minimumConfidence_ > 1 || minIntervalSeconds_ <= 0)
      return Status::failure("invalid constraint manager configuration");
    return Status::success();
  }
  Status initialize(ModuleContext& ctx) override {
    bus_ = &ctx.bus;
    ctx.bus.subscribe<ReplayControl>("navigation.replay.control", [this](const ReplayControl& r) { if (r.reset) lastPublished_ = -1.0; });
    ctx.bus.subscribe<DynamicStatus>(inputTopic_, [this](const auto& status) { onStatus(status); });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override {}
private:
  void onStatus(const DynamicStatus& s) {
    if (s.state == "unknown" || s.confidence < minimumConfidence_ || s.gps_seconds - lastPublished_ < minIntervalSeconds_) return;
    std::vector<ConstraintObservation> observations;
    if (s.state == "stationary" && zuptEnabled_) {
      observations.push_back({s.gps_seconds, s.epoch_id, "zupt", s.confidence});
      if (zpcEnabled_) observations.push_back({s.gps_seconds, s.epoch_id, "zpc", s.confidence});
      if (zhcEnabled_) observations.push_back({s.gps_seconds, s.epoch_id, "zhc", s.confidence});
    }
    if (s.state == "moving" && !s.high_dynamic && nhcEnabled_)
      observations.push_back({s.gps_seconds, s.epoch_id, "nhc", s.confidence});
    if (observations.empty()) return;
    lastPublished_ = s.gps_seconds;
    bus_->publish(outputTopic_, std::move(observations));
  }
  MessageBus* bus_ = nullptr;
  std::string inputTopic_, outputTopic_;
  double minimumConfidence_ = 0.5, minIntervalSeconds_ = 1.0, lastPublished_ = -1.0;
  bool zuptEnabled_ = false, nhcEnabled_ = false, zpcEnabled_ = false, zhcEnabled_ = false;
};
std::unique_ptr<IModule> makeDynamicStatus() { return std::make_unique<DynamicStatusModule>(); }
std::unique_ptr<IModule> makeConstraintManager() { return std::make_unique<ConstraintManagerModule>(); }
}
void registerConstraintModules(ModuleRegistry& r) {
  r.add("dynamic_status", &makeDynamicStatus);
  r.add("constraint_manager", &makeConstraintManager);
}
} // namespace navcore
