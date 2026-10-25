#include "navigation/strapdown_mechanizer.hpp"
#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>

namespace navcore {
namespace {
constexpr double kPi = 3.14159265358979323846;
Eigen::Vector3d vector3(const Json& value, const std::string& name) {
  const auto& a = value.asArray();
  if (a.size() != 3) throw std::runtime_error(name + " must contain exactly 3 numbers");
  return {a[0].asNumber(), a[1].asNumber(), a[2].asNumber()};
}

class InsNavigator final : public IModule {
public:
  std::string_view type() const noexcept override { return "ins_navigator"; }
  Status configure(const Json& cfg) override {
    imuTopic_ = cfg.at("imu_topic").asString();
    solutionTopic_ = cfg.at("solution_topic").asString();
    const auto& paths = cfg.at("paths");
    const auto& ins = cfg.at("ins");
    std::ifstream coordinates(paths.at("rover_coordinates").asString());
    Eigen::Vector3d ecef;
    if (!(coordinates >> ecef.x() >> ecef.y() >> ecef.z())) return Status::failure("cannot read rover ECEF coordinates");
    initial_.lla_rad_m = navigation::ecefToLla(ecef);
    initial_.velocity_ned_mps = vector3(ins.at("initial_velocity_ned_mps"), "initial_velocity_ned_mps");
    initial_.euler_rpy_rad = vector3(ins.at("initial_attitude_rpy_deg"), "initial_attitude_rpy_deg") * (kPi / 180.0);
    initial_.gyro_bias_radps = vector3(ins.at("initial_gyro_bias_radps"), "initial_gyro_bias_radps");
    initial_.accel_bias_mps2 = vector3(ins.at("initial_accel_bias_mps2"), "initial_accel_bias_mps2");
    return Status::success();
  }
  Status initialize(ModuleContext& ctx) override {
    bus_ = &ctx.bus;
    mechanizer_ = std::make_unique<navigation::StrapdownMechanizer>(initial_);
    ctx.bus.subscribe<ImuSample>(imuTopic_, [this](const ImuSample& sample) {
      const auto solution = mechanizer_->update(sample);
      if (solution) bus_->publish(solutionTopic_, *solution);
    });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override { mechanizer_.reset(); }
private:
  MessageBus* bus_ = nullptr;
  std::string imuTopic_, solutionTopic_;
  navigation::InsInitialState initial_;
  std::unique_ptr<navigation::StrapdownMechanizer> mechanizer_;
};

class NavigationCsvSink final : public IModule {
public:
  std::string_view type() const noexcept override { return "navigation_csv_sink"; }
  Status configure(const Json& cfg) override {
    topic_ = cfg.at("subscribe_topic").asString();
    outputPath_ = cfg.at("paths").at("output").asString() + "/" + cfg.at("file_name").asString();
    return Status::success();
  }
  Status initialize(ModuleContext& ctx) override {
    ctx.bus.subscribe<NavigationSolution>(topic_, [this](const NavigationSolution& s) {
      if (out_) out_ << s.gps_seconds << ',' << s.lla_rad_m.x() * 180.0 / kPi << ',' << s.lla_rad_m.y() * 180.0 / kPi << ',' << s.lla_rad_m.z()
                     << ',' << s.ecef_m.x() << ',' << s.ecef_m.y() << ',' << s.ecef_m.z()
                     << ',' << s.velocity_ned_mps.x() << ',' << s.velocity_ned_mps.y() << ',' << s.velocity_ned_mps.z()
                     << ',' << s.q_body_to_ned.w() << ',' << s.q_body_to_ned.x() << ',' << s.q_body_to_ned.y() << ',' << s.q_body_to_ned.z()
                     << ',';
      if (out_) {
        if (s.position_3d_rms_1sigma_m) out_ << *s.position_3d_rms_1sigma_m;
        out_ << ',';
        if (s.velocity_3d_rms_1sigma_mps) out_ << *s.velocity_3d_rms_1sigma_mps;
        out_ << ',';
        if (s.attitude_3d_rms_1sigma_deg) out_ << *s.attitude_3d_rms_1sigma_deg;
        out_ << '\n';
      }
    });
    return Status::success();
  }
  Status start() override {
    try {
      const auto parent = std::filesystem::path(outputPath_).parent_path();
      std::filesystem::create_directories(parent);
      out_.open(outputPath_, std::ios::out | std::ios::trunc);
      if (!out_) return Status::failure("cannot open navigation output: " + outputPath_);
      out_ << "gps_seconds,latitude_deg,longitude_deg,height_m,ecef_x_m,ecef_y_m,ecef_z_m,velocity_n_mps,velocity_e_mps,velocity_d_mps,q_w,q_x,q_y,q_z,position_3d_rms_1sigma_m,velocity_3d_rms_1sigma_mps,attitude_3d_rms_1sigma_deg\n";
      out_.precision(12);
      return Status::success();
    } catch (const std::exception& e) { return Status::failure(e.what()); }
  }
  void stop() noexcept override { if (out_.is_open()) out_.close(); }
private:
  std::string topic_, outputPath_;
  std::ofstream out_;
};

std::unique_ptr<IModule> makeInsNavigator() { return std::make_unique<InsNavigator>(); }
std::unique_ptr<IModule> makeNavigationCsvSink() { return std::make_unique<NavigationCsvSink>(); }
}
void registerNavigatorModule(ModuleRegistry& r) {
  r.add("ins_navigator", &makeInsNavigator);
  r.add("navigation_csv_sink", &makeNavigationCsvSink);
}
}
