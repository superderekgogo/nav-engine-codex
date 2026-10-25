#include "navcore/gnss_preprocessor.hpp"
#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"

#include <iostream>
#include <memory>

namespace navcore {
namespace {

class GnssCycleSlipPreprocessorModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "gnss_cycle_slip_preprocessor"; }

  Status configure(const Json& cfg) override {
    inputTopic_ = cfg.at("subscribe_topic").asString();
    outputTopic_ = cfg.at("publish_topic").asString();
    const auto& options = cfg.at("cycle_slip_preprocessing");
    GnssCycleSlipConfig config;
    config.maximum_epoch_gap_seconds = options.at("maximum_epoch_gap_seconds").asNumber();
    config.doppler_phase_threshold_cycles = options.at("doppler_phase_threshold_cycles").asNumber();
    config.integer_cycle_tolerance_cycles = options.at("integer_cycle_tolerance_cycles").asNumber();
    config.geometry_free_threshold_m = options.at("geometry_free_threshold_m").asNumber();
    config.l1_wavelength_m = options.at("l1_wavelength_m").asNumber();
    config.l2_wavelength_m = options.at("l2_wavelength_m").asNumber();
    config.minimum_common_satellites = static_cast<std::size_t>(options.at("minimum_common_satellites").asNumber());
    preprocessor_ = std::make_unique<GnssCycleSlipPreprocessor>(config);
    return Status::success();
  }

  Status initialize(ModuleContext& context) override {
    bus_ = &context.bus;
    context.bus.subscribe<PreparedGnssEpoch>(inputTopic_, [this](const PreparedGnssEpoch& epoch) {
      auto marked = preprocessor_->process(epoch);
      for (const auto& satellite : marked.satellites) {
        if (satellite.rover_l1_cycle_slip || satellite.base_l1_cycle_slip ||
            satellite.rover_l2_cycle_slip || satellite.base_l2_cycle_slip) {
          if (shownEvents_ < 12) {
            std::cout << "[cycle-slip] t=" << marked.gps_seconds << " svid=" << satellite.svid
                      << " rover(L1/L2)=" << satellite.rover_l1_cycle_slip << '/' << satellite.rover_l2_cycle_slip
                      << " base(L1/L2)=" << satellite.base_l1_cycle_slip << '/' << satellite.base_l2_cycle_slip << std::endl;
            ++shownEvents_;
          }
        }
      }
      bus_->publish(outputTopic_, marked);
    });
    context.bus.subscribe<SystemEvent>("system.events", [this](const SystemEvent& event) {
      if (event.name != "DatasetReplayComplete") return;
      const auto& summary = preprocessor_->summary();
      std::cout << "[cycle-slip] epochs=" << summary.input_epochs
                << ", L1 flags=" << summary.flagged_l1 << " (rover=" << summary.flagged_rover_l1
                << ", base=" << summary.flagged_base_l1 << ")"
                << ", L2 flags=" << summary.flagged_l2 << " (rover=" << summary.flagged_rover_l2
                << ", base=" << summary.flagged_base_l2 << ")"
                << ", unconfirmed L1 candidates rejected=" << summary.rejected_l1_candidates
                << ", event details shown=" << shownEvents_ << std::endl;
    });
    return Status::success();
  }

  Status start() override { return Status::success(); }
  void stop() noexcept override { preprocessor_.reset(); }

private:
  MessageBus* bus_ = nullptr;
  std::string inputTopic_, outputTopic_;
  std::unique_ptr<GnssCycleSlipPreprocessor> preprocessor_;
  std::size_t shownEvents_ = 0;
};

std::unique_ptr<IModule> makeGnssCycleSlipPreprocessor() {
  return std::make_unique<GnssCycleSlipPreprocessorModule>();
}

} // namespace

void registerGnssPreprocessingModules(ModuleRegistry& registry) {
  registry.add("gnss_cycle_slip_preprocessor", &makeGnssCycleSlipPreprocessor);
}

} // namespace navcore
