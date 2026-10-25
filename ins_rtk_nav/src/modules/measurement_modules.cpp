#include "navcore/measurement_models.hpp"
#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace navcore {
namespace {
class TdDdBuilderModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "td_ddcp_builder"; }
  Status configure(const Json& cfg) override {
    inputTopic_ = cfg.at("subscribe_topic").asString();
    outputTopic_ = cfg.at("publish_topic").asString();
    summaryTopic_ = cfg.at("summary_topic").asString();
    const auto& options = cfg.at("fusion_model").at("td_ddcp");
    TdDdBuilderConfig builderConfig;
    builderConfig.max_epoch_gap_seconds = options.at("max_epoch_gap_seconds").asNumber();
    builderConfig.l1_wavelength_m = options.at("wavelength_l1_m").asNumber();
    builderConfig.l2_wavelength_m = options.at("wavelength_l2_m").asNumber();
    builderConfig.phase_sigma_cycles = options.at("phase_sigma_cycles_per_receiver").asNumber();
    builderConfig.doppler_phase_rate_sign = options.at("doppler_phase_rate_sign").asNumber();
    builderConfig.cycle_slip_threshold_cycles = options.at("cycle_slip_threshold_cycles").asNumber();
    builderConfig.minimum_common_satellites = static_cast<std::size_t>(options.at("minimum_common_satellites").asNumber());
    builder_ = std::make_unique<TdDdCarrierPhaseBuilder>(builderConfig);
    return Status::success();
  }
  Status initialize(ModuleContext& context) override {
    bus_ = &context.bus;
    context.bus.subscribe<PreparedGnssEpoch>(inputTopic_, [this](const PreparedGnssEpoch& epoch) {
      ++receivedEpochs_;
      const auto prepared = builder_->push(epoch);
      if (prepared) bus_->publish(outputTopic_, *prepared);
    });
    context.bus.subscribe<SystemEvent>("system.events", [this](const SystemEvent& event) {
      if (event.name == "DatasetReplayComplete") bus_->publish(summaryTopic_, builder_->summary());
    });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override { builder_.reset(); }
private:
  MessageBus* bus_ = nullptr;
  std::string inputTopic_, outputTopic_, summaryTopic_;
  std::unique_ptr<TdDdCarrierPhaseBuilder> builder_;
  std::uint64_t receivedEpochs_ = 0;
};

class TdDdCsvSink final : public IModule {
public:
  std::string_view type() const noexcept override { return "td_ddcp_csv_sink"; }
  Status configure(const Json& cfg) override {
    topic_ = cfg.at("subscribe_topic").asString();
    summaryTopic_ = cfg.at("summary_topic").asString();
    outputPath_ = std::filesystem::path(cfg.at("paths").at("output").asString()) / cfg.at("file_name").asString();
    return Status::success();
  }
  Status initialize(ModuleContext& context) override {
    context.bus.subscribe<PreparedTdDdCarrierPhaseEpoch>(topic_, [this](const PreparedTdDdCarrierPhaseEpoch& epoch) {
      if (!out_) return;
      for (std::size_t i = 0; i < epoch.observations.size(); ++i) {
        const auto& observation = epoch.observations[i];
        out_ << epoch.sequence << ',' << epoch.previous_gps_seconds << ',' << epoch.gps_seconds << ','
             << (observation.band == CarrierPhaseBand::L1 ? "L1" : "L2") << ','
             << observation.svid << ',' << observation.pivot_svid << ',' << observation.wavelength_m << ','
             << observation.delta_double_difference_m << ',' << epoch.covariance_m2(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) << ','
             << (observation.cycle_slip_screened ? 1 : 0) << ','
             << (observation.cycle_slip_detected ? 1 : 0) << ','
             << observation.doppler_phase_residual_cycles << ','
             << observation.satellite_position_previous_ecef_m.x() << ',' << observation.satellite_position_previous_ecef_m.y() << ',' << observation.satellite_position_previous_ecef_m.z() << ','
             << observation.satellite_position_current_ecef_m.x() << ',' << observation.satellite_position_current_ecef_m.y() << ',' << observation.satellite_position_current_ecef_m.z() << ','
             << observation.pivot_position_previous_ecef_m.x() << ',' << observation.pivot_position_previous_ecef_m.y() << ',' << observation.pivot_position_previous_ecef_m.z() << ','
             << observation.pivot_position_current_ecef_m.x() << ',' << observation.pivot_position_current_ecef_m.y() << ',' << observation.pivot_position_current_ecef_m.z() << ','
             << observation.elevation_previous_deg.value_or(-1.0) << ',' << observation.elevation_current_deg.value_or(-1.0) << '\n';
      }
    });
    context.bus.subscribe<TdDdBuildSummary>(summaryTopic_, [](const TdDdBuildSummary& summary) {
      std::cout << "[td-ddcp] input epochs=" << summary.input_epochs << ", output epochs=" << summary.output_epochs
                << ", observations=" << summary.output_observations << ", priming=" << summary.priming_epochs
                << ", slip-screened=" << summary.cycle_slip_screened_observations
                << ", slip-rejected=" << summary.cycle_slip_rejected_observations
                << ", gap resets=" << summary.rejected_gap_pairs << ", pivot resets=" << summary.pivot_reset_pairs
                << ", no phase rows=" << summary.epochs_without_valid_phase << std::endl;
    });
    return Status::success();
  }
  Status start() override {
    try {
      std::filesystem::create_directories(outputPath_.parent_path());
      out_.open(outputPath_, std::ios::out | std::ios::trunc);
      if (!out_) return Status::failure("cannot open TD-DDCP output: " + outputPath_.string());
      out_ << "sequence,previous_gps_seconds,gps_seconds,band,svid,pivot_svid,wavelength_m,delta_double_difference_m,variance_m2,cycle_slip_screened,cycle_slip_detected,doppler_phase_residual_cycles,prev_sat_x_m,prev_sat_y_m,prev_sat_z_m,curr_sat_x_m,curr_sat_y_m,curr_sat_z_m,prev_pivot_x_m,prev_pivot_y_m,prev_pivot_z_m,curr_pivot_x_m,curr_pivot_y_m,curr_pivot_z_m,prev_elevation_deg,curr_elevation_deg\n";
      out_ << std::setprecision(12);
      return Status::success();
    } catch (const std::exception& e) { return Status::failure(e.what()); }
  }
  void stop() noexcept override { if (out_.is_open()) out_.close(); }
private:
  std::string topic_, summaryTopic_ = "measurement.td_ddcp.summary";
  std::filesystem::path outputPath_;
  std::ofstream out_;
};

std::unique_ptr<IModule> makeTdDdBuilder() { return std::make_unique<TdDdBuilderModule>(); }
std::unique_ptr<IModule> makeTdDdCsvSink() { return std::make_unique<TdDdCsvSink>(); }
}

void registerMeasurementModules(ModuleRegistry& registry) {
  registry.add("td_ddcp_builder", &makeTdDdBuilder);
  registry.add("td_ddcp_csv_sink", &makeTdDdCsvSink);
}
} // namespace navcore
