#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"
#include "navcore/state_estimator.hpp"
#include "navigation/strapdown_mechanizer.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace navcore {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr Eigen::Index kStateSize = 18;
constexpr Eigen::Index kAnchorSize = 4; // static ECEF position error and static heading error
constexpr Eigen::Index kAnchoredStateSize = kStateSize + kAnchorSize;

Eigen::Vector3d vector3(const Json& value, const std::string& key) {
  const auto& a = value.asArray();
  if (a.size() != 3) throw std::runtime_error(key + " must contain exactly 3 numbers");
  return {a[0].asNumber(), a[1].asNumber(), a[2].asNumber()};
}
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
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}
double wrapAngle(double angle) { return std::remainder(angle, 2.0 * kPi); }
double headingOf(const Eigen::Quaterniond& input) {
  const Eigen::Quaterniond q = input.normalized();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
Eigen::Vector3d readEcef(const std::string& path) {
  std::ifstream in(path);
  Eigen::Vector3d value;
  if (!(in >> value.x() >> value.y() >> value.z()) || !value.allFinite())
    throw std::runtime_error("cannot read base ECEF coordinate: " + path);
  return value;
}

struct Endpoint {
  ImuSample imu;
  NavigationSolution state;
  Eigen::MatrixXd covariance;
};

class EstimationModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "estimation"; }

  Status configure(const Json& cfg) override {
    const auto& fusion = cfg.at("fusion_model");
    const auto& model = fusion.at("estimation");
    mode_ = fusion.at("combination_mode").asString();
    if (mode_ != "lc" && mode_ != "tc" && mode_ != "ltc") return Status::failure("unsupported estimator combination mode");
    snapshotTopic_ = model.at("snapshot_topic").asString();
    measurementTopic_ = model.at("measurement_topic").asString();
    feedbackTopic_ = model.at("feedback_topic").asString();
    feedbackAppliedTopic_ = model.at("feedback_applied_topic").asString();
    headingLockTopic_ = model.at("heading_lock_topic").asString();
    replayControlTopic_ = model.at("replay_control_topic").asString();
    outputTopic_ = model.at("estimator_output_topic").asString();
    dynamicStatusTopic_ = model.at("dynamic_status_topic").asString();
    innovationPath_ = std::filesystem::path(cfg.at("paths").at("output").asString()) / model.at("innovations_file").asString();
    baseEcef_ = readEcef(cfg.at("paths").at("base_coordinates").asString());
    const auto& ins = cfg.at("ins");
    gyroBiasInitial_ = vector3(ins.at("initial_gyro_bias_radps"), "initial_gyro_bias_radps");
    initialPositionStd_ = vector3(model.at("initial_position_std_m"), "initial_position_std_m");
    initialVelocityStd_ = vector3(model.at("initial_velocity_std_mps"), "initial_velocity_std_mps");
    initialAttitudeStd_ = vector3(model.at("initial_attitude_std_deg"), "initial_attitude_std_deg") * kPi / 180.0;
    initialGyroBiasStd_ = vector3(model.at("initial_gyro_bias_std_radps"), "initial_gyro_bias_std_radps");
    initialAccelBiasStd_ = vector3(model.at("initial_accel_bias_std_mps2"), "initial_accel_bias_std_mps2");
    initialLeverArmStd_ = vector3(model.at("initial_lever_arm_std_m"), "initial_lever_arm_std_m");
    gyroNoiseDensity_ = model.at("gyro_noise_density_radps_sqrt_hz").asNumber();
    accelNoiseDensity_ = model.at("accel_noise_density_mps2_sqrt_hz").asNumber();
    gyroBiasRandomWalk_ = model.at("gyro_bias_random_walk_radps_sqrt_s").asNumber();
    accelBiasRandomWalk_ = model.at("accel_bias_random_walk_mps2_sqrt_s").asNumber();
    leverArmRandomWalk_ = model.at("lever_arm_random_walk_m_sqrt_s").asNumber();
    minimumVariance_ = model.at("minimum_measurement_variance").asNumber();
    nisGateLc_ = model.at("nis_gate_lc").asNumber();
    nisGateTc_ = model.at("nis_gate_tc").asNumber();
    nisGateLtc_ = model.at("nis_gate_ltc").asNumber();
    estimateLeverArm_ = model.at("estimate_lever_arm").asBool();
    leverArmMinRotationRad_ = model.at("lever_arm_min_rotation_deg").asNumber() * kPi / 180.0;
    floatScale_ = model.at("float_measurement_variance_scale").asNumber();
    fixScale_ = model.at("fix_measurement_variance_scale").asNumber();
    tdScale_ = model.at("td_ddcp_variance_scale").asNumber();
    const auto& constraints = fusion.at("constraints");
    zuptSigma_ = constraints.at("zupt_sigma_mps").asNumber();
    nhcSigmaLateral_ = constraints.at("nhc_sigma_lateral_mps").asNumber();
    nhcSigmaVertical_ = constraints.at("nhc_sigma_vertical_mps").asNumber();
    constraintNisGate_ = constraints.at("nis_gate").asNumber();
    zpcSigma_ = constraints.at("zero_position_change_sigma_m").asNumber();
    zhcSigma_ = constraints.at("zero_heading_change_sigma_deg").asNumber() * kPi / 180.0;
    acceptFloat_ = model.at("accept_float").asBool();
    acceptFix_ = model.at("accept_fix").asBool();
    for (const auto& item : fusion.at("measurement_models").asArray()) {
      const auto& id = item.asString();
      if (id == "rtk_position_velocity_loose_v1") { rtkPositionEnabled_ = true; rtkVelocityEnabled_ = true; }
      if (id == "rtk_position_loose_v1") rtkPositionEnabled_ = true;
      if (id == "td_ddcp_tight_v1") useTcModel_ = true;
      if (id == "l1_tdcp_delta_position_velocity_v1") useLtcTdcpModel_ = true;
      if (id == "zupt_stationary_v1") useZupt_ = true;
      if (id == "vehicle_nhc_v1") useNhc_ = true;
      if (id == "zero_position_change_v1") useZpc_ = true;
      if (id == "zero_heading_change_v1") useZhc_ = true;
    }
    if ((initialPositionStd_.array() <= 0).any() || (initialVelocityStd_.array() <= 0).any() ||
        (initialAttitudeStd_.array() <= 0).any() || (initialGyroBiasStd_.array() <= 0).any() ||
        (initialAccelBiasStd_.array() <= 0).any() || (initialLeverArmStd_.array() <= 0).any() ||
        minimumVariance_ <= 0 || nisGateLc_ < 0 || nisGateTc_ < 0 || nisGateLtc_ < 0 || leverArmMinRotationRad_ <= 0 || floatScale_ <= 0 || fixScale_ <= 0 || tdScale_ <= 0 ||
        gyroNoiseDensity_ < 0 || accelNoiseDensity_ < 0 || gyroBiasRandomWalk_ < 0 || accelBiasRandomWalk_ < 0 || leverArmRandomWalk_ < 0 ||
        zuptSigma_ <= 0 || nhcSigmaLateral_ <= 0 || nhcSigmaVertical_ <= 0 ||
        zpcSigma_ <= 0 || zhcSigma_ <= 0 || constraintNisGate_ < 0 ||
        ((useZpc_ || useZhc_) && !useZupt_))
      return Status::failure("invalid estimation noise or covariance configuration");
    const std::string stateModel = fusion.at("state_model").asString();
    if (stateModel != "ins_error_18d_lever_arm_v1") return Status::failure("state_model must be ins_error_18d_lever_arm_v1");
    return Status::success();
  }

  Status initialize(ModuleContext& context) override {
    bus_ = &context.bus;
    context.bus.subscribe<ReplayControl>(replayControlTopic_, [this](const ReplayControl& control) {
      if (control.reset) resetFilter();
    });
    context.bus.subscribe<InertialStateSnapshot>(snapshotTopic_, [this](const auto& snapshot) { onSnapshot(snapshot); });
    context.bus.subscribe<NavigationMeasurementBundle>(measurementTopic_, [this](const auto& bundle) { onMeasurement(bundle); });
    context.bus.subscribe<FeedbackApplied>(feedbackAppliedTopic_, [this](const auto& feedback) { onFeedback(feedback); });
    context.bus.subscribe<DynamicStatus>(dynamicStatusTopic_, [this](const auto& status) { onDynamicStatus(status); });
    context.bus.subscribe<SystemEvent>("system.events", [this](const SystemEvent& event) {
      if (event.name == "NavigationReplayComplete") printSummary();
    });
    return Status::success();
  }

  Status start() override {
    try {
      std::filesystem::create_directories(innovationPath_.parent_path());
      innovations_.open(innovationPath_, std::ios::out | std::ios::trunc);
      if (!innovations_) return Status::failure("cannot open innovations output: " + innovationPath_.string());
      innovations_ << "gps_seconds,previous_gps_seconds,mode,measurement,signal,component,svid,pivot_svid,observed,predicted,innovation,sigma,normalized_innovation,nis,dof,accepted\n";
      innovations_ << std::setprecision(12);
      return Status::success();
    } catch (const std::exception& error) { return Status::failure(error.what()); }
  }
  void stop() noexcept override { if (innovations_.is_open()) innovations_.close(); endpoints_.clear(); }

private:
  void resetFilter() {
    if (headingLockActive_ && bus_)
      bus_->publish(headingLockTopic_, HeadingLockControl{false, headingLockId_, headingLockRad_, headingLockSigmaRad_, headingLockReferenceLla_});
    Eigen::VectorXd x = Eigen::VectorXd::Zero(kStateSize);
    Eigen::MatrixXd p = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
    for (int axis = 0; axis < 3; ++axis) {
      p(axis, axis) = std::pow(initialPositionStd_[axis], 2);
      p(axis + 3, axis + 3) = std::pow(initialVelocityStd_[axis], 2);
      p(axis + 6, axis + 6) = std::pow(initialAttitudeStd_[axis], 2);
      p(axis + 9, axis + 9) = std::pow(initialGyroBiasStd_[axis], 2);
      p(axis + 12, axis + 12) = std::pow(initialAccelBiasStd_[axis], 2);
      p(axis + 15, axis + 15) = std::pow(initialLeverArmStd_[axis], 2);
    }
    estimator_.initialize(x, p);
    anchorValid_ = false;
    anchorCreatedEpoch_ = 0;
    headingLockActive_ = false;
    headingLockId_ = 0;
    headingLockRad_ = headingLockSigmaRad_ = 0.0;
    headingLockReferenceLla_.setZero();
    endpoints_.clear();
    current_.reset();
    pending_ = false;
    accumulatedRotationRad_ = 0.0;
    leverObservable_ = false;
    updatesRtk_ = updatesTd_ = rejectedTd_ = rowsTd_ = 0;
  }

  void onSnapshot(const InertialStateSnapshot& snapshot) {
    if (!estimator_.initialized()) resetFilter();
    if (current_ && snapshot.epoch_id == current_->epoch_id && std::abs(snapshot.gps_seconds - current_->gps_seconds) <= 1e-9) {
      current_ = snapshot;
      if (!endpoints_.empty() && std::abs(endpoints_.back().state.gps_seconds - snapshot.gps_seconds) <= 1e-9) {
        endpoints_.back().state = snapshot.predicted_state;
        endpoints_.back().imu = snapshot.imu;
        endpoints_.back().covariance = currentCovariance();
      }
      return;
    }
    double dt = 0.0;
    if (current_) {
      dt = snapshot.gps_seconds - current_->gps_seconds;
      if (!(dt > 0.0) || dt > 1.0) throw std::runtime_error("invalid IMU dt in EstimationModule");
      const Eigen::Quaterniond relative = current_->predicted_state.q_body_to_ned.conjugate() * snapshot.predicted_state.q_body_to_ned;
      accumulatedRotationRad_ += 2.0 * std::acos(std::clamp(std::abs(relative.normalized().w()), 0.0, 1.0));
      leverObservable_ = estimateLeverArm_ && accumulatedRotationRad_ >= leverArmMinRotationRad_;
    }
    const Eigen::Index filterSize = estimator_.state().size();
    Eigen::MatrixXd phi = Eigen::MatrixXd::Identity(filterSize, filterSize);
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(filterSize, filterSize);
    if (dt > 0.0) {
      const Eigen::Matrix3d cBn = snapshot.predicted_state.q_body_to_ned.toRotationMatrix();
      const Eigen::Vector3d forceNed = cBn * (snapshot.imu.specific_force_mps2 - snapshot.predicted_state.accel_bias_mps2);
      const Eigen::Vector3d omegaBody = snapshot.imu.gyro_radps - snapshot.predicted_state.gyro_bias_radps;
      phi.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
      phi.block<3, 3>(3, 6) = -skew(forceNed) * dt;
      phi.block<3, 3>(3, 12) = -cBn * dt;
      phi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(cBn * omegaBody) * dt;
      phi.block<3, 3>(6, 9) = -cBn * dt;
      const double aq = accelNoiseDensity_ * accelNoiseDensity_;
      q.block<3, 3>(0, 0).diagonal().setConstant(aq * dt * dt * dt / 3.0);
      q.block<3, 3>(0, 3).diagonal().setConstant(aq * dt * dt / 2.0);
      q.block<3, 3>(3, 0) = q.block<3, 3>(0, 3);
      q.block<3, 3>(3, 3).diagonal().setConstant(aq * dt);
      q.block<3, 3>(6, 6).diagonal().setConstant(gyroNoiseDensity_ * gyroNoiseDensity_ * dt);
      q.block<3, 3>(9, 9).diagonal().setConstant(gyroBiasRandomWalk_ * gyroBiasRandomWalk_ * dt);
      q.block<3, 3>(12, 12).diagonal().setConstant(accelBiasRandomWalk_ * accelBiasRandomWalk_ * dt);
      q.block<3, 3>(15, 15).diagonal().setConstant(leverArmRandomWalk_ * leverArmRandomWalk_ * dt);
    }
    estimator_.predict(phi, q);
    current_ = snapshot;
    endpoints_.push_back(Endpoint{snapshot.imu, snapshot.predicted_state, currentCovariance()});
    while (!endpoints_.empty() && snapshot.gps_seconds - endpoints_.front().state.gps_seconds > 2.0) endpoints_.pop_front();
    publishOutput(snapshot.gps_seconds, snapshot.epoch_id);
  }

  void onMeasurement(const NavigationMeasurementBundle& bundle) {
    if (!current_ || bundle.epoch_id != current_->epoch_id) return;
    if (bundle.rtk && (rtkPositionEnabled_ || rtkVelocityEnabled_)) updateRtk(*bundle.rtk, bundle.epoch_id);
    if (mode_ != "lc" && bundle.td_ddcp) {
      if (mode_ == "tc" && useTcModel_) updateTc(*bundle.td_ddcp, bundle.epoch_id);
      else if (mode_ == "ltc" && useLtcTdcpModel_) updateLtc(*bundle.td_ddcp, bundle.epoch_id);
    }
    bool zuptAccepted = false;
    for (const auto& constraint : bundle.constraints) {
      if (constraint.epoch_id != bundle.epoch_id || std::abs(constraint.gps_seconds - bundle.gps_seconds) > 1e-6) continue;
      if (constraint.type == "zupt" && useZupt_) zuptAccepted = updateZupt(constraint, bundle.epoch_id);
      else if (constraint.type == "nhc" && useNhc_) updateNhc(constraint, bundle.epoch_id);
      else if (constraint.type == "zpc" && useZpc_ && zuptAccepted)
        updateZpc(constraint, bundle.epoch_id);
      // ZHC constrains heading change, which is independent of whether the
      // position anchor passes its own NIS gate.  A rejected ZPC must not
      // suppress an otherwise valid zero-heading update.
      else if (constraint.type == "zhc" && useZhc_ && zuptAccepted) {
        if (headingLockActive_) logHeadingLockProjection(constraint);
        else if (updateZhc(constraint, bundle.epoch_id)) activateHeadingLock();
      }
    }
  }

  bool updateZupt(const ConstraintObservation& observation, std::uint64_t epochId) {
    const Eigen::VectorXd z = Eigen::Vector3d::Zero();
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, kStateSize);
    h.block<3, 3>(0, 3).setIdentity();
    const Eigen::VectorXd innovation = -current_->predicted_state.velocity_ned_mps;
    const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * zuptSigma_ * zuptSigma_;
    return submitUpdate(observation.gps_seconds, observation.gps_seconds, "zupt", "NED", {0, 1, 2}, z, h, r,
                        innovation, epochId, nullptr, nullptr, false, constraintNisGate_ > 0 ? std::optional<double>(constraintNisGate_) : std::nullopt);
  }

  void updateNhc(const ConstraintObservation& observation, std::uint64_t epochId) {
    const auto& state = current_->predicted_state;
    const Eigen::Matrix3d cBn = state.q_body_to_ned.toRotationMatrix().transpose();
    Eigen::Matrix<double, 2, 3> select;
    select << 0, 1, 0, 0, 0, 1; // FRD right and down; forward speed is unconstrained.
    const Eigen::Vector2d predicted = select * cBn * state.velocity_ned_mps;
    const Eigen::VectorXd z = Eigen::Vector2d::Zero();
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, kStateSize);
    h.block<2, 3>(0, 3) = select * cBn;
    // The navigator injects the attitude error on the left in NED.
    h.block<2, 3>(0, 6) = select * cBn * skew(state.velocity_ned_mps);
    Eigen::Matrix2d r = Eigen::Matrix2d::Zero();
    r(0, 0) = nhcSigmaLateral_ * nhcSigmaLateral_;
    r(1, 1) = nhcSigmaVertical_ * nhcSigmaVertical_;
    submitUpdate(observation.gps_seconds, observation.gps_seconds, "nhc", "FRD", {0, 1}, z, h, r,
                 -predicted, epochId, nullptr, nullptr, false, constraintNisGate_ > 0 ? std::optional<double>(constraintNisGate_) : std::nullopt);
  }

  Eigen::MatrixXd currentCovariance() const {
    return estimator_.covariance().topLeftCorner(kStateSize, kStateSize);
  }
  Eigen::MatrixXd expandJacobian(const Eigen::MatrixXd& h) const {
    if (h.cols() == estimator_.state().size()) return h;
    if (h.cols() != kStateSize) throw std::invalid_argument("measurement Jacobian has an invalid state dimension");
    Eigen::MatrixXd expanded = Eigen::MatrixXd::Zero(h.rows(), estimator_.state().size());
    expanded.leftCols(kStateSize) = h;
    return expanded;
  }
  Eigen::VectorXd currentErrorState() const { return estimator_.state().head(kStateSize); }
  void ensureStaticAnchor() {
    if (anchorValid_) return;
    if (!current_ || estimator_.state().size() != kStateSize)
      throw std::logic_error("cannot initialize a static anchor in the current filter state");
    const Eigen::MatrixXd p = estimator_.covariance();
    Eigen::Matrix<double, kAnchorSize, kStateSize> clone = Eigen::Matrix<double, kAnchorSize, kStateSize>::Zero();
    const Eigen::Matrix3d currentNedToEcef = ecefToNedRotation(current_->predicted_state.lla_rad_m).transpose();
    clone.block<3, 3>(0, 0) = currentNedToEcef;
    constexpr double eps = 1e-6;
    const double nominalHeading = headingOf(current_->predicted_state.q_body_to_ned);
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Quaterniond perturbation(Eigen::AngleAxisd(eps, Eigen::Vector3d::Unit(axis)));
      clone(3, 6 + axis) = wrapAngle(headingOf(perturbation * current_->predicted_state.q_body_to_ned) - nominalHeading) / eps;
    }
    Eigen::VectorXd augmentedState = Eigen::VectorXd::Zero(kAnchoredStateSize);
    Eigen::MatrixXd augmentedCovariance = Eigen::MatrixXd::Zero(kAnchoredStateSize, kAnchoredStateSize);
    augmentedCovariance.topLeftCorner(kStateSize, kStateSize) = p;
    const Eigen::MatrixXd cross = p * clone.transpose();
    augmentedCovariance.topRightCorner(kStateSize, kAnchorSize) = cross;
    augmentedCovariance.bottomLeftCorner(kAnchorSize, kStateSize) = cross.transpose();
    augmentedCovariance.bottomRightCorner(kAnchorSize, kAnchorSize) = clone * p * clone.transpose();
    // A cloned variable needs a tiny independent component to keep the joint
    // covariance positive definite; physical anchor tolerance is represented by R_zpc/R_zhc.
    augmentedCovariance.bottomRightCorner(kAnchorSize, kAnchorSize).diagonal() +=
      (Eigen::Vector4d() << 1e-8, 1e-8, 1e-8, 1e-10).finished();
    estimator_.initialize(augmentedState, augmentedCovariance);
    anchorPositionEcef_ = current_->predicted_state.ecef_m;
    anchorHeadingRad_ = nominalHeading;
    anchorCreatedEpoch_ = current_->epoch_id;
    anchorValid_ = true;
  }
  void onDynamicStatus(const DynamicStatus& status) {
    if (anchorValid_ && status.state != "stationary") {
      const bool releaseLock = headingLockActive_;
      const HeadingLockControl released{false, headingLockId_, headingLockRad_, headingLockSigmaRad_, headingLockReferenceLla_};
      headingLockActive_ = false;
      const Eigen::VectorXd state = estimator_.state().head(kStateSize);
      const Eigen::MatrixXd covariance = currentCovariance();
      estimator_.initialize(state, covariance);
      anchorValid_ = false;
      anchorCreatedEpoch_ = 0;
      if (releaseLock) bus_->publish(headingLockTopic_, released);
    }
  }
  double headingSigmaDeg(const Eigen::Quaterniond& q, const Eigen::Matrix3d& attitudeCovariance) const {
    constexpr double eps = 1e-6;
    const double nominal = headingOf(q);
    Eigen::RowVector3d jacobian;
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Quaterniond perturbation(Eigen::AngleAxisd(eps, Eigen::Vector3d::Unit(axis)));
      jacobian[axis] = wrapAngle(headingOf(perturbation * q) - nominal) / eps;
    }
    return std::sqrt(std::max(0.0, (jacobian * attitudeCovariance * jacobian.transpose())(0, 0))) * 180.0 / kPi;
  }
  void activateHeadingLock() {
    if (headingLockActive_ || !current_) return;
    headingLockActive_ = true;
    ++headingLockId_;
    headingLockRad_ = headingOf(current_->predicted_state.q_body_to_ned);
    headingLockSigmaRad_ = headingSigmaDeg(current_->predicted_state.q_body_to_ned,
                                            currentCovariance().block<3, 3>(6, 6)) * kPi / 180.0;
    headingLockReferenceLla_ = current_->predicted_state.lla_rad_m;
    bus_->publish(headingLockTopic_, HeadingLockControl{true, headingLockId_, headingLockRad_, headingLockSigmaRad_, headingLockReferenceLla_});
  }
  void logHeadingLockProjection(const ConstraintObservation& observation) {
    if (!current_ || !headingLockActive_ || !innovations_) return;
    const double appliedDeg = current_->predicted_state.heading_lock_applied_correction_deg.value_or(0.0);
    const double innovation = -appliedDeg * kPi / 180.0;
    const double variance = std::max(headingLockSigmaRad_ * headingLockSigmaRad_, minimumVariance_);
    EstimatorUpdateResult result;
    result.accepted = true;
    result.innovation = Eigen::VectorXd::Constant(1, innovation);
    result.innovation_covariance = Eigen::MatrixXd::Constant(1, 1, variance);
    result.normalized_innovation = Eigen::VectorXd::Constant(1, innovation / std::sqrt(variance));
    result.nis = innovation * innovation / variance;
    logRow(observation.gps_seconds, observation.gps_seconds, "zhc_lock_projection", "heading_lock", "heading_change_rad", -1, -1,
           0.0, -innovation, innovation, result, 0, 1, true);
  }
  bool updateZpc(const ConstraintObservation& observation, std::uint64_t epochId) {
    ensureStaticAnchor();
    if (anchorCreatedEpoch_ == epochId) return true; // the anchor epoch defines zero displacement
    const auto& state = current_->predicted_state;
    const Eigen::Matrix3d anchorEcefToNed = ecefToNedRotation(navigation::ecefToLla(anchorPositionEcef_));
    const Eigen::Vector3d predicted = anchorEcefToNed * (state.ecef_m - anchorPositionEcef_);
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, kAnchoredStateSize);
    h.block<3, 3>(0, 0) = anchorEcefToNed * ecefToNedRotation(state.lla_rad_m).transpose();
    h.block<3, 3>(0, kStateSize) = -anchorEcefToNed;
    const Eigen::VectorXd z = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * zpcSigma_ * zpcSigma_;
    return submitUpdate(observation.gps_seconds, observation.gps_seconds, "zpc", "anchor_NED", {0, 1, 2}, z, h, r,
                        -predicted - h * estimator_.state(), epochId, nullptr, nullptr, false,
                        constraintNisGate_ > 0 ? std::optional<double>(constraintNisGate_) : std::nullopt);
  }
  bool updateZhc(const ConstraintObservation& observation, std::uint64_t epochId) {
    ensureStaticAnchor();
    if (anchorCreatedEpoch_ == epochId) return true;
    const double predicted = wrapAngle(headingOf(current_->predicted_state.q_body_to_ned) - anchorHeadingRad_);
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(1, kAnchoredStateSize);
    constexpr double eps = 1e-6;
    const double nominalHeading = headingOf(current_->predicted_state.q_body_to_ned);
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Quaterniond perturbation(Eigen::AngleAxisd(eps, Eigen::Vector3d::Unit(axis)));
      h(0, 6 + axis) = wrapAngle(headingOf(perturbation * current_->predicted_state.q_body_to_ned) - nominalHeading) / eps;
    }
    h(0, kStateSize + 3) = -1.0;
    const Eigen::VectorXd z = Eigen::VectorXd::Zero(1);
    const Eigen::MatrixXd r = Eigen::MatrixXd::Constant(1, 1, zhcSigma_ * zhcSigma_);
    return submitUpdate(observation.gps_seconds, observation.gps_seconds, "zhc", "anchor_heading", {0}, z, h, r,
                        Eigen::VectorXd::Constant(1, -predicted) - h * estimator_.state(), epochId, nullptr, nullptr, false,
                        constraintNisGate_ > 0 ? std::optional<double>(constraintNisGate_) : std::nullopt);
  }

  const Endpoint* findEndpoint(double timestamp) const {
    const Endpoint* best = nullptr;
    double distance = 1e100;
    for (const auto& endpoint : endpoints_) {
      const double d = std::abs(endpoint.state.gps_seconds - timestamp);
      if (d < distance) { distance = d; best = &endpoint; }
    }
    return distance <= 0.011 ? best : nullptr;
  }

  Eigen::Matrix<double, 3, kStateSize> antennaPositionJacobian(const NavigationSolution& s) const {
    Eigen::Matrix<double, 3, kStateSize> j = Eigen::Matrix<double, 3, kStateSize>::Zero();
    const Eigen::Vector3d leverN = s.q_body_to_ned * s.lever_arm_body_m;
    j.block<3, 3>(0, 0).setIdentity();
    // Attitude error is injected on the left: d(C*l) = dtheta x lever.
    j.block<3, 3>(0, 6) = -skew(leverN);
    j.block<3, 3>(0, 15) = s.q_body_to_ned.toRotationMatrix();
    return j;
  }
  Eigen::Vector3d antennaPositionEcef(const NavigationSolution& s) const {
    const Eigen::Matrix3d ecefToNed = ecefToNedRotation(s.lla_rad_m);
    return s.ecef_m + ecefToNed.transpose() * (s.q_body_to_ned * s.lever_arm_body_m);
  }
  Eigen::Matrix<double, 3, kStateSize> antennaVelocityJacobian(const Endpoint& endpoint) const {
    const auto& s = endpoint.state;
    const Eigen::Matrix3d cBn = s.q_body_to_ned.toRotationMatrix();
    const Eigen::Vector3d omega = endpoint.imu.gyro_radps - s.gyro_bias_radps;
    const Eigen::Vector3d rotationalVelocity = cBn * omega.cross(s.lever_arm_body_m);
    Eigen::Matrix<double, 3, kStateSize> j = Eigen::Matrix<double, 3, kStateSize>::Zero();
    j.block<3, 3>(0, 3).setIdentity();
    j.block<3, 3>(0, 6) = -skew(rotationalVelocity);
    // omega = measured_rate - gyro_bias; bias error is true minus nominal.
    j.block<3, 3>(0, 9) = cBn * skew(s.lever_arm_body_m);
    j.block<3, 3>(0, 15) = cBn * skew(omega);
    return j;
  }
  Eigen::Vector3d antennaVelocityNed(const Endpoint& e) const {
    return e.state.velocity_ned_mps + e.state.q_body_to_ned * (e.imu.gyro_radps - e.state.gyro_bias_radps).cross(e.state.lever_arm_body_m);
  }

  void updateRtk(const RtkPositionVelocityObservation& obs, std::uint64_t epochId) {
    const bool fix = obs.solution_status == "FIX";
    const bool floating = obs.solution_status == "FLOAT";
    if ((!fix || !acceptFix_) && (!floating || !acceptFloat_)) return;
    const auto& state = current_->predicted_state;
    const Eigen::Matrix3d rot = ecefToNedRotation(state.lla_rad_m);
    const Eigen::Matrix3d rToNed = rot;
    Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
    transform.block<3, 3>(0, 0) = rToNed;
    transform.block<3, 3>(3, 3) = rToNed;
    const Eigen::Matrix<double, 6, 6> rFull = transform * obs.covariance * transform.transpose() * (fix ? fixScale_ : floatScale_);
    const Endpoint endpoint{current_->imu, state, estimator_.covariance()};
    const Eigen::Vector3d predictedPosition = rToNed * (obs.antenna_position_ecef_m - antennaPositionEcef(state));
    const Eigen::Vector3d predictedVelocity = rToNed * obs.antenna_velocity_ecef_mps - antennaVelocityNed(endpoint);
    Eigen::Matrix<double, 6, 1> observed;
    observed.head<3>() = predictedPosition;
    observed.tail<3>() = predictedVelocity;
    const Eigen::Matrix<double, 3, kStateSize> pJacNed = antennaPositionJacobian(state);
    Eigen::Matrix<double, 3, kStateSize> pJac = pJacNed;
    Eigen::Matrix<double, 3, kStateSize> vJacNed = antennaVelocityJacobian(endpoint);
    if (!leverObservable_) { pJac.block<3, 3>(0, 15).setZero(); vJacNed.block<3, 3>(0, 15).setZero(); }
    std::vector<int> components;
    for (int c = 0; c < 6; ++c) {
      const bool enabled = c < 3 ? rtkPositionEnabled_ : rtkVelocityEnabled_;
      if (enabled && (obs.valid_components & (1u << c)) != 0) components.push_back(c);
    }
    if (components.empty()) return;
    const Eigen::Index m = static_cast<Eigen::Index>(components.size());
    Eigen::VectorXd z(m);
    Eigen::MatrixXd h(m, kStateSize), r(m, m);
    for (Eigen::Index row = 0; row < m; ++row) {
      const int c = components[static_cast<std::size_t>(row)];
      z[row] = observed[c];
      h.row(row) = c < 3 ? pJac.row(c) : vJacNed.row(c - 3);
      for (Eigen::Index col = 0; col < m; ++col)
        r(row, col) = rFull(c, components[static_cast<std::size_t>(col)]);
    }
    const Eigen::VectorXd innovation = z - h * currentErrorState();
    submitUpdate(obs.gps_seconds, obs.gps_seconds, "rtk_pv", "mixed", components, z, h, r, innovation, epochId, &obs);
  }

  static double range(const Eigen::Vector3d& r, const Eigen::Vector3d& s) { return (s - r).norm(); }
  double ddGeometry(const Eigen::Vector3d& receiver, const TdDdCarrierPhaseObservation& o, bool current) const {
    const auto& sat = current ? o.satellite_position_current_ecef_m : o.satellite_position_previous_ecef_m;
    const auto& pivot = current ? o.pivot_position_current_ecef_m : o.pivot_position_previous_ecef_m;
    return range(receiver, sat) - range(baseEcef_, sat) - range(receiver, pivot) + range(baseEcef_, pivot);
  }
  Eigen::RowVector3d ddGradientEcef(const Eigen::Vector3d& receiver, const Eigen::Vector3d& sat, const Eigen::Vector3d& pivot) const {
    return ((receiver - sat) / (receiver - sat).norm() - (receiver - pivot) / (receiver - pivot).norm()).transpose();
  }

  void updateTc(const PreparedTdDdCarrierPhaseEpoch& epoch, std::uint64_t epochId) {
    const Endpoint* previous = findEndpoint(epoch.previous_gps_seconds);
    if (!previous || epoch.observations.empty() || epoch.covariance_m2.rows() != static_cast<Eigen::Index>(epoch.observations.size())) {
      ++rejectedTd_;
      return;
    }
    const auto& state = current_->predicted_state;
    const Eigen::Vector3d currentAntenna = antennaPositionEcef(state);
    const Eigen::Vector3d previousAntenna = antennaPositionEcef(previous->state);
    const Eigen::Matrix3d currentR = ecefToNedRotation(state.lla_rad_m);
    const Eigen::Matrix3d previousR = ecefToNedRotation(previous->state.lla_rad_m);
    Eigen::Matrix<double, 3, kStateSize> currentJ = currentR.transpose() * antennaPositionJacobian(state);
    Eigen::Matrix<double, 3, kStateSize> previousJ = previousR.transpose() * antennaPositionJacobian(previous->state);
    if (!leverObservable_) { currentJ.block<3, 3>(0, 15).setZero(); previousJ.block<3, 3>(0, 15).setZero(); }
    const Eigen::Index m = static_cast<Eigen::Index>(epoch.observations.size());
    Eigen::VectorXd z(m), predicted(m);
    Eigen::MatrixXd h(m, kStateSize), hPrev(m, kStateSize);
    for (Eigen::Index i = 0; i < m; ++i) {
      const auto& o = epoch.observations[static_cast<std::size_t>(i)];
      z[i] = o.delta_double_difference_m;
      predicted[i] = ddGeometry(currentAntenna, o, true) - ddGeometry(previousAntenna, o, false);
      const auto currentGradient = ddGradientEcef(currentAntenna,
        o.satellite_position_current_ecef_m, o.pivot_position_current_ecef_m);
      const auto previousGradient = ddGradientEcef(previousAntenna,
        o.satellite_position_previous_ecef_m, o.pivot_position_previous_ecef_m);
      h.row(i) = currentGradient * currentJ;
      hPrev.row(i) = -previousGradient * previousJ;
    }
    Eigen::MatrixXd r = epoch.covariance_m2 * tdScale_ + hPrev * previous->covariance * hPrev.transpose();
    r.diagonal().array() += minimumVariance_;
    const Eigen::VectorXd innovation = z - predicted - h * currentErrorState();
    std::vector<int> rows(static_cast<std::size_t>(m));
    for (int i = 0; i < m; ++i) rows[static_cast<std::size_t>(i)] = i;
    if (submitUpdate(epoch.gps_seconds, epoch.previous_gps_seconds, "td_ddcp_range", "mixed", rows, z, h, r, innovation, epochId, nullptr, &epoch)) {
      ++updatesTd_;
      rowsTd_ += static_cast<std::size_t>(m);
    } else ++rejectedTd_;
  }

  void updateLtc(const PreparedTdDdCarrierPhaseEpoch& epoch, std::uint64_t epochId) {
    const Endpoint* previous = findEndpoint(epoch.previous_gps_seconds);
    const double dt = epoch.gps_seconds - epoch.previous_gps_seconds;
    if (!previous || !(dt > 0.0) || epoch.observations.empty() ||
        epoch.covariance_m2.rows() != static_cast<Eigen::Index>(epoch.observations.size())) { ++rejectedTd_; return; }
    std::vector<Eigen::Index> l1;
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(epoch.observations.size()); ++i)
      if (epoch.observations[static_cast<std::size_t>(i)].band == CarrierPhaseBand::L1) l1.push_back(i);
    if (l1.size() < 4) { ++rejectedTd_; return; }
    const Eigen::Index m = static_cast<Eigen::Index>(l1.size());
    const Eigen::Vector3d r0 = antennaPositionEcef(previous->state);
    Eigen::MatrixXd a(m, 3), previousSensitivity(m, 3), rPhase(m, m);
    Eigen::VectorXd observed(m), geom0(m);
    for (Eigen::Index i = 0; i < m; ++i) {
      const Eigen::Index source = l1[static_cast<std::size_t>(i)];
      const auto& o = epoch.observations[static_cast<std::size_t>(source)];
      observed[i] = o.delta_double_difference_m;
      geom0[i] = ddGeometry(r0, o, true) - ddGeometry(r0, o, false);
      a.row(i) = ddGradientEcef(r0, o.satellite_position_current_ecef_m, o.pivot_position_current_ecef_m);
      const Eigen::RowVector3d previousGeom = ddGradientEcef(r0, o.satellite_position_previous_ecef_m, o.pivot_position_previous_ecef_m);
      previousSensitivity.row(i) = a.row(i) - previousGeom;
      for (Eigen::Index j = 0; j < m; ++j)
        rPhase(i, j) = epoch.covariance_m2(source, l1[static_cast<std::size_t>(j)]) * tdScale_;
    }
    rPhase.diagonal().array() += minimumVariance_;
    const Eigen::Matrix3d prevEn = ecefToNedRotation(previous->state.lla_rad_m);
    Eigen::Matrix<double, 3, kStateSize> jAntennaPrev = prevEn.transpose() * antennaPositionJacobian(previous->state);
    if (!leverObservable_) jAntennaPrev.block<3, 3>(0, 15).setZero();
    const Eigen::Matrix3d previousAntCov = jAntennaPrev * previous->covariance * jAntennaPrev.transpose();
    rPhase += previousSensitivity * previousAntCov * previousSensitivity.transpose();
    Eigen::LDLT<Eigen::MatrixXd> phaseFactor(0.5 * (rPhase + rPhase.transpose()));
    if (phaseFactor.info() != Eigen::Success || !phaseFactor.isPositive()) { ++rejectedTd_; return; }
    const Eigen::Matrix3d information = a.transpose() * phaseFactor.solve(a);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(information);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues()[0] <= eigen.eigenvalues()[2] * 1e-8) { ++rejectedTd_; return; }
    Eigen::LDLT<Eigen::Matrix3d> infoFactor(information);
    if (infoFactor.info() != Eigen::Success || !infoFactor.isPositive()) { ++rejectedTd_; return; }
    const Eigen::Matrix3d covDelta = infoFactor.solve(Eigen::Matrix3d::Identity());
    const Eigen::Vector3d deltaEcef = covDelta * a.transpose() * phaseFactor.solve(observed - geom0);
    const Eigen::Matrix3d currentEn = ecefToNedRotation(current_->predicted_state.lla_rad_m);
    const Eigen::Vector3d measuredVelocity = currentEn * deltaEcef / dt;
    Eigen::Matrix3d rVelocity = currentEn * covDelta * currentEn.transpose() / (dt * dt);
    Eigen::Matrix<double, 3, kStateSize> jVelocityPrev = antennaVelocityJacobian(*previous);
    if (!leverObservable_) jVelocityPrev.block<3, 3>(0, 15).setZero();
    rVelocity += 0.25 * jVelocityPrev * previous->covariance * jVelocityPrev.transpose();
    const Endpoint currentEndpoint{current_->imu, current_->predicted_state, currentCovariance()};
    const Eigen::Vector3d predictedVelocity = 0.5 * (antennaVelocityNed(*previous) + antennaVelocityNed(currentEndpoint));
    Eigen::Matrix<double, 3, kStateSize> h = 0.5 * antennaVelocityJacobian(currentEndpoint);
    if (!leverObservable_) h.block<3, 3>(0, 15).setZero();
    const Eigen::VectorXd z = measuredVelocity;
    const Eigen::VectorXd innovation = z - predictedVelocity - h * currentErrorState();
    std::vector<int> components{0, 1, 2};
    const bool accepted = submitUpdate(epoch.gps_seconds, epoch.previous_gps_seconds, "l1_tdcp_delta_position_velocity", "L1", components,
                                       z, h, rVelocity, innovation, epochId, nullptr, &epoch, true);
    if (accepted) { ++updatesTd_; rowsTd_ += l1.size(); } else ++rejectedTd_;
  }

  bool submitUpdate(double gps, double previousGps, const std::string& measurement, const std::string& signal,
                    const std::vector<int>& labels, const Eigen::VectorXd& z, const Eigen::MatrixXd& h,
                    const Eigen::MatrixXd& r, const Eigen::VectorXd& innovation, std::uint64_t epochId,
                    const RtkPositionVelocityObservation* rtk = nullptr,
                    const PreparedTdDdCarrierPhaseEpoch* td = nullptr, bool isVelocity = false,
                    std::optional<double> gateOverride = std::nullopt) {
    if (pending_) return false;
    const Eigen::VectorXd oldState = estimator_.state();
    const Eigen::MatrixXd oldCovariance = estimator_.covariance();
    EstimatorUpdateResult result;
    try {
      const double gate = gateOverride.value_or(mode_ == "lc" ? nisGateLc_ : (mode_ == "tc" ? nisGateTc_ : nisGateLtc_));
      const std::optional<double> gateOption = gate > 0.0 ? std::optional<double>(gate) : std::nullopt;
      const Eigen::MatrixXd expandedH = expandJacobian(h);
      result = estimator_.update(innovation, expandedH, r, EstimatorUpdateOptions{gateOption, minimumVariance_});
    } catch (const std::exception& error) {
      std::cerr << "[estimation] " << measurement << " rejected at " << gps << ": " << error.what() << std::endl;
      return false;
    }
    const bool accepted = result.accepted;
    if (rtk) {
      for (std::size_t i = 0; i < labels.size(); ++i) {
        const int c = labels[i];
        const char* names[] = {"position_n_m", "position_e_m", "position_d_m", "velocity_n_mps", "velocity_e_mps", "velocity_d_mps"};
        const double predicted = z[static_cast<Eigen::Index>(i)] - innovation[static_cast<Eigen::Index>(i)];
        logRow(gps, previousGps, measurement, signal, names[c], -1, -1, z[static_cast<Eigen::Index>(i)], predicted,
               innovation[static_cast<Eigen::Index>(i)], result, static_cast<Eigen::Index>(i), static_cast<int>(labels.size()), accepted);
      }
    } else if (td && !isVelocity) {
      for (Eigen::Index i = 0; i < z.size(); ++i) {
        const auto& o = td->observations[static_cast<std::size_t>(i)];
        const double predicted = z[i] - innovation[i];
        logRow(gps, previousGps, measurement, o.band == CarrierPhaseBand::L1 ? "L1" : "L2", "range_m", o.svid, o.pivot_svid,
               z[i], predicted, innovation[i], result, i, static_cast<int>(z.size()), accepted);
      }
    } else if (td && isVelocity) {
      const char* names[] = {"velocity_n_mps", "velocity_e_mps", "velocity_d_mps"};
      for (int axis = 0; axis < 3; ++axis) {
        const double predicted = z[axis] - innovation[axis];
        logRow(gps, previousGps, measurement, signal, names[axis], -1, td->pivot_l1_svid, z[axis], predicted,
               innovation[axis], result, axis, 3, accepted);
      }
    } else if (measurement == "zupt" || measurement == "nhc") {
      const char* zuptNames[] = {"velocity_n_mps", "velocity_e_mps", "velocity_d_mps"};
      const char* nhcNames[] = {"velocity_right_mps", "velocity_down_mps"};
      for (Eigen::Index i = 0; i < z.size(); ++i) {
        const std::string component = measurement == "zupt" ? zuptNames[i] : nhcNames[i];
        logRow(gps, previousGps, measurement, signal, component, -1, -1, z[i], z[i] - innovation[i], innovation[i],
               result, i, static_cast<int>(z.size()), accepted);
      }
    } else if (measurement == "zpc" || measurement == "zhc") {
      const char* zpcNames[] = {"position_n_m", "position_e_m", "position_d_m"};
      const char* zhcNames[] = {"heading_change_rad"};
      for (Eigen::Index i = 0; i < z.size(); ++i) {
        const std::string component = measurement == "zpc" ? zpcNames[i] : zhcNames[i];
        logRow(gps, previousGps, measurement, signal, component, -1, -1, z[i], z[i] - innovation[i], innovation[i],
               result, i, static_cast<int>(z.size()), accepted);
      }
    }
    if (!accepted) {
      estimator_.initialize(oldState, oldCovariance);
      return false;
    }
    if (rtk) ++updatesRtk_;
    pending_ = true;
    pendingEpoch_ = epochId;
    pendingGps_ = current_->gps_seconds;
    pendingCorrection_ = estimator_.state();
    rollbackState_ = oldState;
    rollbackCovariance_ = oldCovariance;
    bus_->publish(feedbackTopic_, ErrorStateFeedback{pendingGps_, pendingEpoch_, pendingCorrection_.head(kStateSize)});
    return true;
  }

  void onFeedback(const FeedbackApplied& result) {
    if (!pending_ || result.epoch_id != pendingEpoch_ || std::abs(result.gps_seconds - pendingGps_) > 1e-6) return;
    if (!result.accepted) estimator_.initialize(rollbackState_, rollbackCovariance_);
    else {
      if (anchorValid_ && pendingCorrection_.size() == kAnchoredStateSize) {
        anchorPositionEcef_ += pendingCorrection_.segment<3>(kStateSize);
        anchorHeadingRad_ = wrapAngle(anchorHeadingRad_ + pendingCorrection_[kStateSize + 3]);
      }
      Eigen::MatrixXd reset = Eigen::MatrixXd::Identity(estimator_.state().size(), estimator_.state().size());
      reset.block<3, 3>(6, 6) -= 0.5 * skew(pendingCorrection_.segment<3>(6));
      estimator_.resetErrorState(reset);
    }
    if (!endpoints_.empty() && current_ && std::abs(endpoints_.back().state.gps_seconds - current_->gps_seconds) <= 1e-9)
      endpoints_.back().covariance = currentCovariance();
    pending_ = false;
    if (current_) publishOutput(current_->gps_seconds, current_->epoch_id);
  }

  void logRow(double gps, double previousGps, const std::string& measurement, const std::string& signal,
              const std::string& component, int svid, int pivot, double observed, double predicted, double innovation,
              const EstimatorUpdateResult& result, Eigen::Index row, int dof, bool accepted) {
    if (!innovations_) return;
    const double sigma = row < result.innovation_covariance.rows() ? std::sqrt(std::max(result.innovation_covariance(row, row), 0.0)) : 0.0;
    const double normalized = row < result.normalized_innovation.size() ? result.normalized_innovation[row] : 0.0;
    innovations_ << gps << ',' << previousGps << ',' << mode_ << ',' << measurement << ',' << signal << ',' << component << ','
                 << svid << ',' << pivot << ',' << observed << ',' << predicted << ',' << innovation << ',' << sigma << ','
                 << normalized << ',' << result.nis << ',' << dof << ',' << (accepted ? 1 : 0) << '\n';
  }

  void publishOutput(double gps, std::uint64_t epochId) {
    const Eigen::MatrixXd p = currentCovariance();
    const Eigen::Matrix3d attitudeCovariance = p.block<3, 3>(6, 6);
    // Error-state attitude is a left-multiplicative rotation in NED. Map its
    // covariance into Euler roll/pitch/heading with a local numerical Jacobian.
    Eigen::Matrix3d eulerJacobian;
    const Eigen::Quaterniond q = current_ ? current_->predicted_state.q_body_to_ned : Eigen::Quaterniond::Identity();
    const auto euler = [](const Eigen::Quaterniond& input) {
      const Eigen::Quaterniond n = input.normalized();
      const double w=n.w(), x=n.x(), y=n.y(), z=n.z();
      return Eigen::Vector3d(
        std::atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y)),
        std::asin(std::clamp(2.0*(w*y-z*x), -1.0, 1.0)),
        std::atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z)));
    };
    constexpr double epsilon = 1e-6;
    const Eigen::Vector3d nominalEuler = euler(q);
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Quaterniond perturbation(Eigen::AngleAxisd(epsilon, Eigen::Vector3d::Unit(axis)));
      const Eigen::Vector3d difference = euler(perturbation * q) - nominalEuler;
      for (int angle = 0; angle < 3; ++angle)
        eulerJacobian(angle, axis) = std::remainder(difference[angle], 2.0*kPi) / epsilon;
    }
    const Eigen::Matrix3d eulerCovariance = eulerJacobian * attitudeCovariance * eulerJacobian.transpose();
    const Eigen::Vector3d eulerSigmaDeg = eulerCovariance.diagonal().cwiseMax(0.0).cwiseSqrt() * (180.0 / kPi);
    const double freeHeadingSigmaDeg = eulerSigmaDeg[2];
    const double outputHeadingSigmaDeg = headingLockActive_ ? headingLockSigmaRad_ * 180.0 / kPi : freeHeadingSigmaDeg;
    const double lockHeadingDeg = headingLockRad_ * 180.0 / kPi;
    const double lockCorrectionDeg = current_ ? current_->predicted_state.heading_lock_applied_correction_deg.value_or(0.0) : 0.0;
    bus_->publish(outputTopic_, EstimatorOutput{gps, epochId,
      std::sqrt(std::max(0.0, p.block(0, 0, 3, 3).trace())),
      std::sqrt(std::max(0.0, p.block(3, 3, 3, 3).trace())),
      std::sqrt(std::max(0.0, attitudeCovariance.trace())) * 180.0 / kPi,
      eulerSigmaDeg[0], eulerSigmaDeg[1], outputHeadingSigmaDeg, freeHeadingSigmaDeg,
      headingLockActive_, headingLockId_, lockHeadingDeg, lockCorrectionDeg});
  }
  void printSummary() const {
    std::cout << "[estimation:" << mode_ << "] RTK updates=" << updatesRtk_ << ", TD updates=" << updatesTd_
              << ", TD rows=" << rowsTd_ << ", TD rejected=" << rejectedTd_;
    if (estimator_.initialized()) std::cout << ", final position 3D 1-sigma=" << std::sqrt(currentCovariance().block(0, 0, 3, 3).trace()) << " m";
    std::cout << std::endl;
  }

  MessageBus* bus_ = nullptr;
  std::string mode_, snapshotTopic_, measurementTopic_, feedbackTopic_, feedbackAppliedTopic_, headingLockTopic_, replayControlTopic_, outputTopic_, dynamicStatusTopic_;
  Eigen::Vector3d baseEcef_ = Eigen::Vector3d::Zero(), gyroBiasInitial_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d initialPositionStd_ = Eigen::Vector3d::Constant(10.0), initialVelocityStd_ = Eigen::Vector3d::Constant(5.0);
  Eigen::Vector3d initialAttitudeStd_ = Eigen::Vector3d::Constant(0.1), initialGyroBiasStd_ = Eigen::Vector3d::Constant(0.01);
  Eigen::Vector3d initialAccelBiasStd_ = Eigen::Vector3d::Constant(0.1), initialLeverArmStd_ = Eigen::Vector3d::Constant(1.0);
  double gyroNoiseDensity_ = 0.003, accelNoiseDensity_ = 0.05, gyroBiasRandomWalk_ = 1e-4, accelBiasRandomWalk_ = 1e-3;
  double leverArmRandomWalk_ = 1e-4, minimumVariance_ = 4e-4;
  double nisGateLc_ = 0.0, nisGateTc_ = 0.0, nisGateLtc_ = 0.0;
  double leverArmMinRotationRad_ = 15.0 * kPi / 180.0, accumulatedRotationRad_ = 0.0;
  double floatScale_ = 1.0, fixScale_ = 1.0, tdScale_ = 1.0;
  bool acceptFloat_ = true, acceptFix_ = true, rtkPositionEnabled_ = false, rtkVelocityEnabled_ = false;
  bool useTcModel_ = false, useLtcTdcpModel_ = false;
  bool leverObservable_ = false, estimateLeverArm_ = false;
  bool useZupt_ = false, useNhc_ = false, useZpc_ = false, useZhc_ = false, anchorValid_ = false, headingLockActive_ = false;
  std::uint64_t headingLockId_ = 0;
  double headingLockRad_ = 0.0, headingLockSigmaRad_ = 0.0;
  Eigen::Vector3d headingLockReferenceLla_ = Eigen::Vector3d::Zero();
  double zuptSigma_ = 0.05, nhcSigmaLateral_ = 0.1, nhcSigmaVertical_ = 0.1, zpcSigma_ = 0.05, zhcSigma_ = 0.5 * kPi / 180.0, constraintNisGate_ = 0.0;
  Eigen::Vector3d anchorPositionEcef_ = Eigen::Vector3d::Zero();
  double anchorHeadingRad_ = 0.0;
  std::uint64_t anchorCreatedEpoch_ = 0;
  StateEstimator estimator_;
  std::optional<InertialStateSnapshot> current_;
  std::deque<Endpoint> endpoints_;
  Eigen::VectorXd pendingCorrection_, rollbackState_;
  Eigen::MatrixXd rollbackCovariance_;
  std::uint64_t pendingEpoch_ = 0;
  double pendingGps_ = 0.0;
  bool pending_ = false;
  std::size_t updatesRtk_ = 0, updatesTd_ = 0, rejectedTd_ = 0, rowsTd_ = 0;
  std::filesystem::path innovationPath_;
  std::ofstream innovations_;
};

std::unique_ptr<IModule> makeEstimation() { return std::make_unique<EstimationModule>(); }
}

void registerEstimationModule(ModuleRegistry& registry) { registry.add("estimation", &makeEstimation); }
} // namespace navcore
