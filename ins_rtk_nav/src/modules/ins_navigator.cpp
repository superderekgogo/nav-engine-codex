#include "navigation/strapdown_mechanizer.hpp"
#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>

namespace navcore {
namespace {
constexpr double kPi = 3.14159265358979323846;
Eigen::Matrix3d ecefToNedRotation(const Eigen::Vector3d& lla) {
  const double lat = lla.x(), lon = lla.y();
  Eigen::Matrix3d r;
  r << -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat),
       -std::sin(lon), std::cos(lon), 0.0,
       -std::cos(lat) * std::cos(lon), -std::cos(lat) * std::sin(lon), -std::sin(lat);
  return r;
}
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0,-v.z(),v.y(), v.z(),0.0,-v.x(), -v.y(),v.x(),0.0;
  return m;
}
Eigen::Vector3d quaternionEulerRpy(const Eigen::Quaterniond& input) {
  const Eigen::Quaterniond q = input.normalized();
  const double w=q.w(), x=q.x(), y=q.y(), z=q.z();
  return {std::atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y)),
          std::asin(std::clamp(2.0*(w*y-z*x), -1.0, 1.0)),
          std::atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))};
}
Eigen::Quaterniond eulerRpyQuaternion(const Eigen::Vector3d& rpy) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).normalized();
}
Eigen::Vector3d vector3(const Json& value, const std::string& name) {
  const auto& a = value.asArray();
  if (a.size() != 3) throw std::runtime_error(name + " must contain exactly 3 numbers");
  return {a[0].asNumber(), a[1].asNumber(), a[2].asNumber()};
}

class InsNavigator final : public IModule {
public:
  std::string_view type() const noexcept override { return "ins_navigator"; }
  Status configure(const Json& cfg) override {
    replayImuTopic_ = cfg.at("replay_imu_topic").asString();
    snapshotTopic_ = cfg.at("snapshot_topic").asString();
    feedbackTopic_ = cfg.at("feedback_topic").asString();
    feedbackAppliedTopic_ = cfg.at("feedback_applied_topic").asString();
    headingLockTopic_ = cfg.at("heading_lock_topic").asString();
    estimatorOutputTopic_ = cfg.at("estimator_output_topic").asString();
    replayControlTopic_ = cfg.at("replay_control_topic").asString();
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
    ctx.bus.subscribe<ReplayControl>(replayControlTopic_, [this](const ReplayControl& control) {
      if (control.reset) { mechanizer_ = std::make_unique<navigation::StrapdownMechanizer>(initial_); epochId_ = 0; latest_.reset(); headingLock_.reset(); }
    });
    ctx.bus.subscribe<ImuSample>(replayImuTopic_, [this](const ImuSample& sample) {
      const auto solution = mechanizer_->update(sample);
      if (!solution) return;
      latest_ = *solution;
      latestImu_ = sample;
      latest_->epoch_id = ++epochId_;
      applyHeadingLock(*latest_);
      if (headingLock_) mechanizer_->setNavigationState(*latest_);
      bus_->publish(snapshotTopic_, InertialStateSnapshot{sample.gps_seconds, epochId_, sample, *latest_});
    });
    ctx.bus.subscribe<ErrorStateFeedback>(feedbackTopic_, [this](const ErrorStateFeedback& feedback) {
      bool accepted = latest_.has_value() && feedback.epoch_id == epochId_ &&
                      std::abs(feedback.gps_seconds - latest_->gps_seconds) <= 1e-6 && feedback.correction.size() == 18;
      if (accepted) {
        const auto cEn = ecefToNedRotation(latest_->lla_rad_m);
        latest_->ecef_m += cEn.transpose() * feedback.correction.segment<3>(0);
        latest_->lla_rad_m = navigation::ecefToLla(latest_->ecef_m);
        latest_->velocity_ned_mps += feedback.correction.segment<3>(3);
        const auto dtheta = feedback.correction.segment<3>(6);
        if (dtheta.norm() > 1e-14)
          latest_->q_body_to_ned = (Eigen::Quaterniond(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized())) * latest_->q_body_to_ned).normalized();
        latest_->lever_arm_body_m += feedback.correction.segment<3>(15);
        mechanizer_->applyErrorStateCorrection(*latest_, feedback.correction.segment<3>(9), feedback.correction.segment<3>(12));
        applyHeadingLock(*latest_);
        if (headingLock_) mechanizer_->setNavigationState(*latest_);
        bus_->publish(snapshotTopic_, InertialStateSnapshot{latest_->gps_seconds, epochId_, latestImu_, *latest_});
      }
      bus_->publish(feedbackAppliedTopic_, FeedbackApplied{feedback.gps_seconds, feedback.epoch_id, accepted});
    });
    ctx.bus.subscribe<HeadingLockControl>(headingLockTopic_, [this](const HeadingLockControl& control) {
      if (control.active) headingLock_ = control;
      else headingLock_.reset();
      if (!latest_) return;
      applyHeadingLock(*latest_);
      if (headingLock_) mechanizer_->setNavigationState(*latest_);
      bus_->publish(snapshotTopic_, InertialStateSnapshot{latest_->gps_seconds, epochId_, latestImu_, *latest_});
      if (!control.active) {
        latest_->heading_lock_active = false;
        latest_->heading_lock_id = control.lock_id;
        bus_->publish(solutionTopic_, *latest_);
      }
    });
    ctx.bus.subscribe<EstimatorOutput>(estimatorOutputTopic_, [this](const EstimatorOutput& output) {
      if (!latest_ || output.epoch_id != epochId_ || std::abs(output.gps_seconds - latest_->gps_seconds) > 1e-6) return;
      latest_->position_3d_rms_1sigma_m = output.position_3d_rms_1sigma_m;
      latest_->velocity_3d_rms_1sigma_mps = output.velocity_3d_rms_1sigma_mps;
      latest_->attitude_3d_rms_1sigma_deg = output.attitude_3d_rms_1sigma_deg;
      latest_->roll_rms_1sigma_deg = output.roll_rms_1sigma_deg;
      latest_->pitch_rms_1sigma_deg = output.pitch_rms_1sigma_deg;
      latest_->heading_rms_1sigma_deg = output.heading_rms_1sigma_deg;
      latest_->free_heading_rms_1sigma_deg = output.free_heading_rms_1sigma_deg;
      latest_->heading_lock_active = output.heading_lock_active;
      latest_->heading_lock_id = output.heading_lock_id;
      latest_->heading_lock_heading_deg = output.heading_lock_heading_deg;
      latest_->heading_lock_applied_correction_deg = output.heading_lock_applied_correction_deg;
      bus_->publish(solutionTopic_, *latest_);
    });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override { mechanizer_.reset(); }
private:
  MessageBus* bus_ = nullptr;
  void applyHeadingLock(NavigationSolution& solution) {
    if (!headingLock_) return;
    const double rawHeading = quaternionEulerRpy(solution.q_body_to_ned).z();
    const Eigen::Matrix3d ecefToCurrentNed = ecefToNedRotation(solution.lla_rad_m);
    const Eigen::Matrix3d ecefToReferenceNed = ecefToNedRotation(headingLock_->reference_lla_rad_m);
    Eigen::Matrix3d bodyToReferenceNed = ecefToReferenceNed * ecefToCurrentNed.transpose() *
                                        solution.q_body_to_ned.toRotationMatrix();
    const Eigen::Vector3d rpy = quaternionEulerRpy(Eigen::Quaterniond(bodyToReferenceNed));
    Eigen::Vector3d lockedRpy = rpy;
    lockedRpy.z() = headingLock_->heading_rad;
    bodyToReferenceNed = eulerRpyQuaternion(lockedRpy).toRotationMatrix();
    const Eigen::Matrix3d bodyToCurrentNed = ecefToCurrentNed * ecefToReferenceNed.transpose() * bodyToReferenceNed;
    solution.q_body_to_ned = Eigen::Quaterniond(bodyToCurrentNed).normalized();
    solution.heading_lock_active = true;
    solution.heading_lock_id = headingLock_->lock_id;
    solution.heading_lock_heading_deg = headingLock_->heading_rad * 180.0 / kPi;
    solution.heading_lock_applied_correction_deg = std::remainder(rawHeading - headingLock_->heading_rad, 2.0*kPi) * 180.0 / kPi;
    solution.heading_rms_1sigma_deg = headingLock_->sigma_rad * 180.0 / kPi;
  }
  std::string replayImuTopic_, snapshotTopic_, feedbackTopic_, feedbackAppliedTopic_, headingLockTopic_, estimatorOutputTopic_, replayControlTopic_, solutionTopic_;
  navigation::InsInitialState initial_;
  std::unique_ptr<navigation::StrapdownMechanizer> mechanizer_;
  std::optional<NavigationSolution> latest_;
  ImuSample latestImu_;
  std::optional<HeadingLockControl> headingLock_;
  std::uint64_t epochId_ = 0;
};

class NavigationCsvSink final : public IModule {
public:
  std::string_view type() const noexcept override { return "navigation_csv_sink"; }
  Status configure(const Json& cfg) override {
    topic_ = cfg.at("subscribe_topic").asString();
    outputPath_ = cfg.at("paths").at("output").asString() + "/" + cfg.at("file_name").asString();
    if (const Json* rate = cfg.find("output_rate_hz")) {
      outputRateHz_ = rate->asNumber();
      if (!std::isfinite(outputRateHz_) || outputRateHz_ <= 0.0)
        return Status::failure("output_rate_hz must be a finite positive number");
      outputIntervalSeconds_ = 1.0 / outputRateHz_;
    }
    return Status::success();
  }
  Status initialize(ModuleContext& ctx) override {
    ctx.bus.subscribe<NavigationSolution>(topic_, [this](const NavigationSolution& s) { solutions_[s.gps_seconds] = s; });
    ctx.bus.subscribe<SystemEvent>("system.events", [this](const SystemEvent& event) {
      if (event.name != "NavigationReplayComplete" || !out_) return;
      std::optional<double> nextOutputTime;
      for (const auto& item : solutions_) {
        const auto& s = item.second;
        if (outputIntervalSeconds_ > 0.0) {
          if (!nextOutputTime) nextOutputTime = s.gps_seconds;
          if (s.gps_seconds + 1e-9 < *nextOutputTime) continue;
          do { *nextOutputTime += outputIntervalSeconds_; }
          while (*nextOutputTime <= s.gps_seconds + 1e-9);
        }
        out_ << s.gps_seconds << ',' << s.lla_rad_m.x() * 180.0 / kPi << ',' << s.lla_rad_m.y() * 180.0 / kPi << ',' << s.lla_rad_m.z()
             << ',' << s.ecef_m.x() << ',' << s.ecef_m.y() << ',' << s.ecef_m.z()
             << ',' << s.velocity_ned_mps.x() << ',' << s.velocity_ned_mps.y() << ',' << s.velocity_ned_mps.z()
             << ',' << s.q_body_to_ned.w() << ',' << s.q_body_to_ned.x() << ',' << s.q_body_to_ned.y() << ',' << s.q_body_to_ned.z()
             << ',';
        if (s.position_3d_rms_1sigma_m) out_ << *s.position_3d_rms_1sigma_m;
        out_ << ',';
        if (s.velocity_3d_rms_1sigma_mps) out_ << *s.velocity_3d_rms_1sigma_mps;
        out_ << ',';
        if (s.attitude_3d_rms_1sigma_deg) out_ << *s.attitude_3d_rms_1sigma_deg;
        out_ << ',';
        if (s.roll_rms_1sigma_deg) out_ << *s.roll_rms_1sigma_deg;
        out_ << ',';
        if (s.pitch_rms_1sigma_deg) out_ << *s.pitch_rms_1sigma_deg;
        out_ << ',';
        if (s.heading_rms_1sigma_deg) out_ << *s.heading_rms_1sigma_deg;
        out_ << ',';
        if (s.free_heading_rms_1sigma_deg) out_ << *s.free_heading_rms_1sigma_deg;
        out_ << ',' << (s.heading_lock_active ? 1 : 0) << ',' << s.heading_lock_id << ',';
        if (s.heading_lock_heading_deg) out_ << *s.heading_lock_heading_deg;
        out_ << ',';
        if (s.heading_lock_applied_correction_deg) out_ << *s.heading_lock_applied_correction_deg;
        out_ << ',' << s.lever_arm_body_m.x() << ',' << s.lever_arm_body_m.y() << ',' << s.lever_arm_body_m.z() << '\n';
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
      out_ << "gps_seconds,latitude_deg,longitude_deg,height_m,ecef_x_m,ecef_y_m,ecef_z_m,velocity_n_mps,velocity_e_mps,velocity_d_mps,q_w,q_x,q_y,q_z,position_3d_rms_1sigma_m,velocity_3d_rms_1sigma_mps,attitude_3d_rms_1sigma_deg,roll_rms_1sigma_deg,pitch_rms_1sigma_deg,heading_rms_1sigma_deg,free_heading_rms_1sigma_deg,heading_lock_active,heading_lock_id,heading_lock_heading_deg,heading_lock_applied_correction_deg,lever_b_x_m,lever_b_y_m,lever_b_z_m\n";
      out_.precision(12);
      return Status::success();
    } catch (const std::exception& e) { return Status::failure(e.what()); }
  }
  void stop() noexcept override { if (out_.is_open()) out_.close(); solutions_.clear(); }
private:
  std::string topic_, outputPath_;
  double outputRateHz_ = 0.0;
  double outputIntervalSeconds_ = 0.0;
  std::ofstream out_;
  std::map<double, NavigationSolution> solutions_;
};

std::unique_ptr<IModule> makeInsNavigator() { return std::make_unique<InsNavigator>(); }
std::unique_ptr<IModule> makeNavigationCsvSink() { return std::make_unique<NavigationCsvSink>(); }
}
void registerNavigatorModule(ModuleRegistry& r) {
  r.add("ins_navigator", &makeInsNavigator);
  r.add("navigation_csv_sink", &makeNavigationCsvSink);
}
}
