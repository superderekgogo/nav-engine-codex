#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace navcore {
namespace {
class FilterManagerModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "filter_manager"; }

  Status configure(const Json& cfg) override {
    const auto& model = cfg.at("fusion_model");
    const auto& manager = model.at("filter_manager");
    mode_ = model.at("combination_mode").asString();
    if (mode_ != "lc" && mode_ != "tc" && mode_ != "ltc") return Status::failure("combination_mode must be lc, tc, or ltc");
    for (const auto& item : model.at("measurement_models").asArray()) {
      const auto& id = item.asString();
      if (id == "rtk_position_velocity_loose_v1") useRtk_ = true;
      if (id == "rtk_position_loose_v1") useRtk_ = true;
      if (id == "td_ddcp_tight_v1") useTd_ = true;
      if (id == "l1_tdcp_delta_position_velocity_v1") useTd_ = true;
    }
    imuTopic_ = manager.at("imu_topic").asString();
    rtkTopic_ = manager.at("rtk_topic").asString();
    tdTopic_ = manager.at("td_ddcp_topic").asString();
    replayImuTopic_ = manager.at("replay_imu_topic").asString();
    replayControlTopic_ = manager.at("replay_control_topic").asString();
    measurementTopic_ = manager.at("measurement_topic").asString();
    tolerance_ = manager.at("epoch_tolerance_seconds").asNumber();
    if (!(tolerance_ > 0.0 && tolerance_ <= 0.02)) return Status::failure("epoch_tolerance_seconds must be in (0, 0.02]");
    return Status::success();
  }

  Status initialize(ModuleContext& context) override {
    bus_ = &context.bus;
    context.bus.subscribe<ImuSample>(imuTopic_, [this](const ImuSample& sample) { imu_.push_back(sample); });
    context.bus.subscribe<RtkPositionVelocityObservation>(rtkTopic_, [this](const auto& observation) { rtk_.push_back(observation); });
    context.bus.subscribe<PreparedTdDdCarrierPhaseEpoch>(tdTopic_, [this](const auto& epoch) { td_.push_back(epoch); });
    context.bus.subscribe<std::vector<ConstraintObservation>>("navigation.constraints", [this](const auto& items) {
      pendingConstraints_.insert(pendingConstraints_.end(), items.begin(), items.end());
    });
    context.bus.subscribe<SystemEvent>("system.events", [this](const SystemEvent& event) {
      if (event.name == "RtkBatchComplete") replay();
    });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override { imu_.clear(); rtk_.clear(); td_.clear(); }

private:
  void replay() {
    if (replayed_) return;
    replayed_ = true;
    std::sort(imu_.begin(), imu_.end(), [](const auto& a, const auto& b) { return a.gps_seconds < b.gps_seconds; });
    std::sort(rtk_.begin(), rtk_.end(), [](const auto& a, const auto& b) { return a.gps_seconds < b.gps_seconds; });
    std::sort(td_.begin(), td_.end(), [](const auto& a, const auto& b) { return a.gps_seconds < b.gps_seconds; });
    bus_->publish(replayControlTopic_, ReplayControl{true, false});
    std::size_t ri = 0, ti = 0, bundles = 0;
    std::uint64_t epochId = 0;
    for (const auto& sample : imu_) {
      bus_->publish(replayImuTopic_, sample);
      ++epochId;
      std::optional<RtkPositionVelocityObservation> rtk;
      std::optional<PreparedTdDdCarrierPhaseEpoch> td;
      if (mode_ != "tc" && useRtk_ && ri < rtk_.size() && std::abs(rtk_[ri].gps_seconds - sample.gps_seconds) <= tolerance_) rtk = rtk_[ri++];
      if (mode_ != "lc" && useTd_ && ti < td_.size() && std::abs(td_[ti].gps_seconds - sample.gps_seconds) <= tolerance_) td = td_[ti++];
      std::vector<ConstraintObservation> constraints;
      for (auto it = pendingConstraints_.begin(); it != pendingConstraints_.end();) {
        if (it->gps_seconds <= sample.gps_seconds + 1e-6) {
          if (std::abs(it->gps_seconds - sample.gps_seconds) <= 1e-6 && it->epoch_id == epochId)
            constraints.push_back(*it);
          it = pendingConstraints_.erase(it);
        } else ++it;
      }
      if (rtk || td || !constraints.empty()) {
        NavigationMeasurementBundle bundle;
        bundle.gps_seconds = sample.gps_seconds;
        bundle.epoch_id = epochId;
        bundle.rtk = std::move(rtk);
        bundle.td_ddcp = std::move(td);
        bundle.constraints = std::move(constraints);
        bus_->publish(measurementTopic_, bundle);
        ++bundles;
      }
    }
    std::cout << "[filter-manager:" << mode_ << "] replayed IMU=" << imu_.size() << ", RTK used=" << ri
              << ", TD-DDCP epochs used=" << ti << ", measurement bundles=" << bundles
              << ", unmatched RTK/TD=" << (rtk_.size() - ri) << '/' << (td_.size() - ti) << std::endl;
    bus_->publish(replayControlTopic_, ReplayControl{false, true});
    bus_->publish("system.events", SystemEvent{"NavigationReplayComplete", "FilterManager completed event-time navigation replay."});
  }

  MessageBus* bus_ = nullptr;
  std::string mode_, imuTopic_, rtkTopic_, tdTopic_, replayImuTopic_, replayControlTopic_, measurementTopic_;
  std::vector<ImuSample> imu_;
  std::vector<RtkPositionVelocityObservation> rtk_;
  std::vector<PreparedTdDdCarrierPhaseEpoch> td_;
  std::vector<ConstraintObservation> pendingConstraints_;
  double tolerance_ = 0.005;
  bool replayed_ = false, useRtk_ = false, useTd_ = false;
};

std::unique_ptr<IModule> makeFilterManager() { return std::make_unique<FilterManagerModule>(); }
}

void registerFilterManagerModule(ModuleRegistry& registry) { registry.add("filter_manager", &makeFilterManager); }
} // namespace navcore
