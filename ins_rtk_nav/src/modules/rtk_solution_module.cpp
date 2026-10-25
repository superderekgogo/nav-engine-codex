#include "navcore/module_manager.hpp"
#include "navcore/message_bus.hpp"
#include "navcore/rtklib_adapter.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

namespace navcore {
namespace {
Eigen::Vector3d readInitialEcef(const std::string& path) {
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream row(line); Eigen::Vector3d xyz;
    if (row >> xyz.x() >> xyz.y() >> xyz.z() && xyz.allFinite() && xyz.norm() > 1.0e6) return xyz;
  }
  throw std::runtime_error("cannot read initial rover ECEF coordinate from " + path);
}
class RtkSolutionModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "rtk_solution"; }
  Status configure(const Json& cfg) override {
    inputTopic_ = cfg.at("subscribe_topic").asString();
    outputTopic_ = cfg.at("publish_topic").asString();
    const auto& settings = cfg.at("rtk_solution");
    if (settings.at("engine").asString() != "rtklib_adapter") return Status::failure("unsupported RTK engine");
    if (settings.at("constellation_mapping").asString() != "GPS_PRN") return Status::failure("current RTKLIB adapter only supports GPS_PRN SVID mapping");
    const auto& acceptance = settings.at("solution_acceptance");
    RtkAdapterConfig adapterConfig;
    adapterConfig.accept_fix = acceptance.at("accept_fix").asBool();
    adapterConfig.accept_float = acceptance.at("accept_float").asBool();
    adapterConfig.minimum_satellites = static_cast<int>(acceptance.at("minimum_satellites").asNumber());
    const auto& ar = settings.at("ambiguity_resolution");
    adapterConfig.frequency_count = static_cast<int>(ar.at("frequency_count").asNumber());
    adapterConfig.use_l2_pseudorange = settings.at("observation_mapping").at("use_p2_for_dual_frequency_rtk").asBool();
    adapterConfig.ambiguity_mode = ar.at("mode").asString();
    adapterConfig.ambiguity_ratio_threshold = ar.at("ratio_threshold").asNumber();
    adapterConfig.minimum_lock_count = static_cast<int>(ar.at("minimum_lock_count").asNumber());
    adapterConfig.minimum_fix_count = static_cast<int>(ar.at("minimum_fix_count").asNumber());
    adapterConfig.initial_rover_position_ecef_m = readInitialEcef(cfg.at("paths").at("rover_coordinates").asString());
    const auto& covariance = settings.at("covariance");
    adapterConfig.float_covariance_scale = covariance.at("float_scale").asNumber();
    adapterConfig.fix_covariance_scale = covariance.at("fix_scale").asNumber();
    adapterConfig.maximum_position_variance_m2 = covariance.at("maximum_position_variance_m2").asNumber();
    adapterConfig.maximum_velocity_variance_m2ps2 = covariance.at("maximum_velocity_variance_m2ps2").asNumber();
    adapter_ = std::make_unique<RtklibAdapter>(adapterConfig);
    return Status::success();
  }
  Status initialize(ModuleContext& context) override {
    bus_ = &context.bus;
    context.bus.subscribe<PreparedGnssEpoch>(inputTopic_, [this](const PreparedGnssEpoch& epoch) { epochs_.push_back(epoch); });
    context.bus.subscribe<SystemEvent>("system.events", [this](const SystemEvent& event) {
      if (event.name != "DatasetReplayComplete") return;
      const auto results = adapter_->solve(epochs_);
      std::size_t valid = 0;
      std::size_t slipFlagsL1 = 0, slipFlagsL2 = 0;
      std::size_t shown = 0;
      for (const auto& epoch : epochs_) for (const auto& satellite : epoch.satellites) {
        slipFlagsL1 += static_cast<std::size_t>(satellite.rover_l1_cycle_slip) + static_cast<std::size_t>(satellite.base_l1_cycle_slip);
        slipFlagsL2 += static_cast<std::size_t>(satellite.rover_l2_cycle_slip) + static_cast<std::size_t>(satellite.base_l2_cycle_slip);
      }
      for (const auto& result : results) {
        if (result.valid) { bus_->publish(outputTopic_, result.observation); ++valid; }
        else if (shown++ < 8) std::cout << "[rtk] rejected t=" << result.observation.gps_seconds << ": " << result.diagnostic << std::endl;
      }
      std::cout << "[rtk] epochs=" << epochs_.size() << ", accepted FLOAT/FIX=" << valid
                << ", preprocessed L1/L2 slip flags=" << slipFlagsL1 << '/' << slipFlagsL2 << std::endl;
      bus_->publish("system.events", SystemEvent{"RtkBatchComplete", "RTK solution batch is ready for LTC fusion."});
    });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override { epochs_.clear(); adapter_.reset(); }
private:
  MessageBus* bus_ = nullptr;
  std::string inputTopic_, outputTopic_;
  std::vector<PreparedGnssEpoch> epochs_;
  std::unique_ptr<RtklibAdapter> adapter_;
};

class RtkCsvSink final : public IModule {
public:
  std::string_view type() const noexcept override { return "rtk_csv_sink"; }
  Status configure(const Json& cfg) override {
    topic_ = cfg.at("subscribe_topic").asString();
    outputPath_ = std::filesystem::path(cfg.at("paths").at("output").asString()) / cfg.at("file_name").asString();
    return Status::success();
  }
  Status initialize(ModuleContext& context) override {
    context.bus.subscribe<RtkPositionVelocityObservation>(topic_, [this](const auto& o) {
      if (!out_) return;
      out_ << std::setprecision(12) << o.gps_seconds << ',' << o.solution_status << ',' << o.used_satellites << ',' << o.ambiguity_ratio;
      for (int i = 0; i < 3; ++i) out_ << ',' << o.antenna_position_ecef_m[i];
      for (int i = 0; i < 3; ++i) out_ << ',' << o.antenna_velocity_ecef_mps[i];
      out_ << ',' << static_cast<int>(o.valid_components);
      for (int i = 0; i < 6; ++i) out_ << ',' << o.covariance(i, i);
      out_ << '\n';
    });
    return Status::success();
  }
  Status start() override {
    std::filesystem::create_directories(outputPath_.parent_path());
    out_.open(outputPath_, std::ios::out | std::ios::trunc);
    if (!out_) return Status::failure("cannot open RTK output: " + outputPath_.string());
    out_ << "gps_seconds,status,satellites,ambiguity_ratio,x_ecef_m,y_ecef_m,z_ecef_m,vx_ecef_mps,vy_ecef_mps,vz_ecef_mps,valid_components,var_x,var_y,var_z,var_vx,var_vy,var_vz\n";
    return Status::success();
  }
  void stop() noexcept override { if (out_.is_open()) out_.close(); }
private:
  std::string topic_;
  std::filesystem::path outputPath_;
  std::ofstream out_;
};
std::unique_ptr<IModule> makeRtkSolution() { return std::make_unique<RtkSolutionModule>(); }
std::unique_ptr<IModule> makeRtkCsvSink() { return std::make_unique<RtkCsvSink>(); }
}
void registerRtkModules(ModuleRegistry& registry) {
  registry.add("rtk_solution", &makeRtkSolution);
  registry.add("rtk_csv_sink", &makeRtkCsvSink);
}
}
