#include "navigation/strapdown_mechanizer.hpp"
#include "navcore/message_bus.hpp"
#include "navcore/module_manager.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>

namespace navcore {
namespace {
constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d vector3(const Json& value, const std::string& key) {
  const auto& a = value.asArray();
  if (a.size() != 3) throw std::runtime_error(key + " must contain exactly 3 numbers");
  return {a[0].asNumber(), a[1].asNumber(), a[2].asNumber()};
}
Eigen::Vector3d readEcef(const std::string& path) {
  std::ifstream input(path);
  Eigen::Vector3d xyz;
  if (!(input >> xyz.x() >> xyz.y() >> xyz.z())) throw std::runtime_error("cannot read base ECEF coordinate: " + path);
  return xyz;
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

class LtcFusionModule final : public IModule {
public:
  std::string_view type() const noexcept override { return "ltc_fusion"; }

  Status configure(const Json& cfg) override {
    const auto& fusion = cfg.at("fusion_model");
    const auto& model = fusion.at("ltc_fusion");
    combinationMode_ = fusion.at("combination_mode").asString();
    if (combinationMode_ != "lc" && combinationMode_ != "tc" && combinationMode_ != "ltc")
      return Status::failure("fusion_model.combination_mode must be one of lc, tc, ltc");
    enabled_ = model.at("enabled").asBool();
    imuTopic_ = model.at("imu_topic").asString();
    rtkTopic_ = model.at("rtk_topic").asString();
    tdDdTopic_ = model.at("td_ddcp_topic").asString();
    tdDdEnabled_ = model.at("td_ddcp_enabled").asBool();
    rtkPositionEnabled_ = model.at("rtk_position_enabled").asBool();
    rtkVelocityEnabled_ = model.at("rtk_velocity_enabled").asBool();
    if (combinationMode_ == "lc" && !rtkPositionEnabled_ && !rtkVelocityEnabled_)
      return Status::failure("LC mode requires at least one RTK P/V measurement component");
    if (combinationMode_ == "tc" && !tdDdEnabled_)
      return Status::failure("TC mode requires td_ddcp_enabled=true");
    if (combinationMode_ == "ltc" && (!rtkPositionEnabled_ || !tdDdEnabled_))
      return Status::failure("LTC mode requires RTK position and L1 TD phase velocity updates");
    outputTopic_ = model.at("output_topic").asString();
    initialPositionStd_ = vector3(model.at("initial_position_std_m"), "initial_position_std_m");
    initialVelocityStd_ = vector3(model.at("initial_velocity_std_mps"), "initial_velocity_std_mps");
    initialAttitudeStd_ = vector3(model.at("initial_attitude_std_deg"), "initial_attitude_std_deg") * (kPi / 180.0);
    initialGyroBiasStd_ = vector3(model.at("initial_gyro_bias_std_radps"), "initial_gyro_bias_std_radps");
    initialAccelBiasStd_ = vector3(model.at("initial_accel_bias_std_mps2"), "initial_accel_bias_std_mps2");
    gyroNoiseDensity_ = model.at("gyro_noise_density_radps_sqrt_hz").asNumber();
    accelNoiseDensity_ = model.at("accel_noise_density_mps2_sqrt_hz").asNumber();
    gyroBiasRandomWalk_ = model.at("gyro_bias_random_walk_radps_sqrt_s").asNumber();
    accelBiasRandomWalk_ = model.at("accel_bias_random_walk_mps2_sqrt_s").asNumber();
    minimumVariance_ = model.at("minimum_measurement_variance").asNumber();
    floatScale_ = model.at("float_measurement_variance_scale").asNumber();
    fixScale_ = model.at("fix_measurement_variance_scale").asNumber();
    tdDdScale_ = model.at("td_ddcp_variance_scale").asNumber();
    acceptFloat_ = model.at("accept_float").asBool();
    acceptFix_ = model.at("accept_fix").asBool();
    if (gyroNoiseDensity_ < 0 || accelNoiseDensity_ < 0 || gyroBiasRandomWalk_ < 0 || accelBiasRandomWalk_ < 0 ||
        minimumVariance_ <= 0 || floatScale_ <= 0 || fixScale_ <= 0 || tdDdScale_ <= 0 ||
        (initialPositionStd_.array() <= 0).any() || (initialVelocityStd_.array() <= 0).any() ||
        (initialAttitudeStd_.array() <= 0).any() || (initialGyroBiasStd_.array() <= 0).any() ||
        (initialAccelBiasStd_.array() <= 0).any())
      return Status::failure("LTC process noise and measurement variance settings must be non-negative/positive");

    const auto& paths = cfg.at("paths");
    baseCoordinatesPath_ = paths.at("base_coordinates").asString();
    std::ifstream coordinates(paths.at("rover_coordinates").asString());
    Eigen::Vector3d ecef;
    if (!(coordinates >> ecef.x() >> ecef.y() >> ecef.z())) return Status::failure("cannot read rover ECEF coordinates for LTC initial state");
    const auto& ins = cfg.at("ins");
    initial_.lla_rad_m = navigation::ecefToLla(ecef);
    initial_.velocity_ned_mps = vector3(ins.at("initial_velocity_ned_mps"), "initial_velocity_ned_mps");
    initial_.euler_rpy_rad = vector3(ins.at("initial_attitude_rpy_deg"), "initial_attitude_rpy_deg") * (kPi / 180.0);
    initial_.gyro_bias_radps = vector3(ins.at("initial_gyro_bias_radps"), "initial_gyro_bias_radps");
    initial_.accel_bias_mps2 = vector3(ins.at("initial_accel_bias_mps2"), "initial_accel_bias_mps2");
    innovationPath_ = std::filesystem::path(paths.at("output").asString()) / model.at("innovations_file").asString();
    return Status::success();
  }

  Status initialize(ModuleContext& context) override {
    bus_ = &context.bus;
    if (!enabled_) return Status::success();
    context.bus.subscribe<ImuSample>(imuTopic_, [this](const ImuSample& sample) { imu_.push_back(sample); });
    context.bus.subscribe<RtkPositionVelocityObservation>(rtkTopic_, [this](const auto& observation) { rtk_.push_back(observation); });
    context.bus.subscribe<PreparedTdDdCarrierPhaseEpoch>(tdDdTopic_, [this](const auto& epoch) { tdDd_.push_back(epoch); });
    context.bus.subscribe<SystemEvent>("system.events", [this](const SystemEvent& event) {
      if (event.name == "RtkBatchComplete") processBatch();
    });
    return Status::success();
  }
  Status start() override {
    try {
      std::filesystem::create_directories(innovationPath_.parent_path());
      innovations_.open(innovationPath_, std::ios::out | std::ios::trunc);
      if (!innovations_) return Status::failure("cannot open LTC innovations output: " + innovationPath_.string());
      innovations_ << "gps_seconds,previous_gps_seconds,mode,measurement,signal,component,svid,pivot_svid,observed,predicted,innovation,sigma,normalized_innovation,nis,dof,accepted\n";
      innovations_ << std::setprecision(12);
      return Status::success();
    } catch (const std::exception& e) { return Status::failure(e.what()); }
  }
  void stop() noexcept override { imu_.clear(); rtk_.clear(); tdDd_.clear(); if (innovations_.is_open()) innovations_.close(); }

private:
  void processBatch() {
    if (processed_ || imu_.empty()) return;
    processed_ = true;
    std::sort(rtk_.begin(), rtk_.end(), [](const auto& a, const auto& b) { return a.gps_seconds < b.gps_seconds; });
    std::sort(tdDd_.begin(), tdDd_.end(), [](const auto& a, const auto& b) { return a.gps_seconds < b.gps_seconds; });
    const auto base = readEcef(baseCoordinatesPath_);
    navigation::StrapdownMechanizer mechanizer(initial_);
    Eigen::Matrix<double, 15, 15> p = Eigen::Matrix<double, 15, 15>::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      p(axis, axis) = initialPositionStd_[axis] * initialPositionStd_[axis];
      p(axis + 3, axis + 3) = initialVelocityStd_[axis] * initialVelocityStd_[axis];
      p(axis + 6, axis + 6) = initialAttitudeStd_[axis] * initialAttitudeStd_[axis];
      p(axis + 9, axis + 9) = initialGyroBiasStd_[axis] * initialGyroBiasStd_[axis];
      p(axis + 12, axis + 12) = initialAccelBiasStd_[axis] * initialAccelBiasStd_[axis];
    }
    Eigen::Matrix<double, 15, 1> error = Eigen::Matrix<double, 15, 1>::Zero();
    std::size_t rtkIndex = 0, tdIndex = 0, updates = 0, tdUpdates = 0, tdRows = 0, tdRejected = 0;
    double previousTime = 0.0;
    struct Endpoint {
      double time = 0.0;
      Eigen::Vector3d ecef = Eigen::Vector3d::Zero();
      Eigen::Vector3d velocity_ned = Eigen::Vector3d::Zero();
      Eigen::Matrix3d p_position_ned = Eigen::Matrix3d::Zero();
      Eigen::Matrix3d p_velocity_ned = Eigen::Matrix3d::Zero();
    };
    std::optional<Endpoint> previousEndpoint;

    for (const auto& sample : imu_) {
      auto stateOpt = mechanizer.update(sample);
      if (!stateOpt) continue;
      auto state = *stateOpt;
      const double dt = previousTime > 0.0 ? std::max(0.0, sample.gps_seconds - previousTime) : 0.0;
      previousTime = sample.gps_seconds;
      Eigen::Matrix<double, 15, 15> f = Eigen::Matrix<double, 15, 15>::Identity();
      const Eigen::Matrix3d cBn = state.q_body_to_ned.toRotationMatrix();
      const Eigen::Vector3d forceNed = cBn * (sample.specific_force_mps2 - mechanizer.accelBias());
      const Eigen::Vector3d omegaBody = sample.gyro_radps - mechanizer.gyroBias();
      f.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
      f.block<3, 3>(3, 6) = -skew(forceNed) * dt;
      f.block<3, 3>(3, 12) = -cBn * dt;
      f.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(cBn * omegaBody) * dt;
      f.block<3, 3>(6, 9) = -cBn * dt;
      Eigen::Matrix<double, 15, 15> q = Eigen::Matrix<double, 15, 15>::Zero();
      const double accelQ = accelNoiseDensity_ * accelNoiseDensity_;
      q.block<3, 3>(0, 0).diagonal().setConstant(accelQ * dt * dt * dt / 3.0);
      q.block<3, 3>(0, 3).diagonal().setConstant(accelQ * dt * dt / 2.0);
      q.block<3, 3>(3, 0) = q.block<3, 3>(0, 3);
      q.block<3, 3>(3, 3).diagonal().setConstant(accelQ * dt);
      q.block<3, 3>(6, 6).diagonal().setConstant(gyroNoiseDensity_ * gyroNoiseDensity_ * dt);
      q.block<3, 3>(9, 9).diagonal().setConstant(gyroBiasRandomWalk_ * gyroBiasRandomWalk_ * dt);
      q.block<3, 3>(12, 12).diagonal().setConstant(accelBiasRandomWalk_ * accelBiasRandomWalk_ * dt);
      error = f * error;
      p = f * p * f.transpose() + q;

      while (combinationMode_ != "tc" && rtkIndex < rtk_.size() && rtk_[rtkIndex].gps_seconds <= sample.gps_seconds + 1e-6) {
        const auto& obs = rtk_[rtkIndex++];
        const bool isFix = obs.solution_status == "FIX";
        const bool isFloat = obs.solution_status == "FLOAT";
        if ((!isFix || !acceptFix_) && (!isFloat || !acceptFloat_)) continue;
        const auto rot = ecefToNedRotation(state.lla_rad_m);
        Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
        transform.block<3, 3>(0, 0) = rot;
        transform.block<3, 3>(3, 3) = rot;
        Eigen::Matrix<double, 6, 1> z;
        z.head<3>() = rot * (obs.antenna_position_ecef_m - state.ecef_m);
        z.tail<3>() = rot * obs.antenna_velocity_ecef_mps - state.velocity_ned_mps;
        Eigen::Matrix<double, 6, 6> r = transform * obs.covariance * transform.transpose();
        r *= isFix ? fixScale_ : floatScale_;

        std::vector<int> valid;
        for (int i = 0; i < 6; ++i) {
          const bool componentEnabled = i < 3 ? rtkPositionEnabled_ : rtkVelocityEnabled_;
          if (componentEnabled && (obs.valid_components & (1u << i)) != 0) valid.push_back(i);
        }
        if (valid.empty()) continue;
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(static_cast<int>(valid.size()), 15);
        Eigen::VectorXd residual(static_cast<int>(valid.size()));
        Eigen::MatrixXd rv(static_cast<int>(valid.size()), static_cast<int>(valid.size()));
        for (std::size_t row = 0; row < valid.size(); ++row) {
          const int component = valid[row];
          h(static_cast<int>(row), component) = 1.0;
          residual[static_cast<int>(row)] = z[component] - error[component];
          for (std::size_t col = 0; col < valid.size(); ++col)
            rv(static_cast<int>(row), static_cast<int>(col)) = r(component, valid[col]);
          rv(static_cast<int>(row), static_cast<int>(row)) = std::max(rv(static_cast<int>(row), static_cast<int>(row)), minimumVariance_);
        }
        const Eigen::MatrixXd s = h * p * h.transpose() + rv;
        Eigen::LDLT<Eigen::MatrixXd> sFactor(s);
        if (sFactor.info() != Eigen::Success) continue;
        const Eigen::MatrixXd k = p * h.transpose() * sFactor.solve(Eigen::MatrixXd::Identity(s.rows(), s.cols()));
        const double nis = residual.dot(sFactor.solve(residual));
        const Eigen::VectorXd normalized = residual.array() / s.diagonal().array().sqrt();
        const char* componentNames[] = {"position_n_m", "position_e_m", "position_d_m", "velocity_n_mps", "velocity_e_mps", "velocity_d_mps"};
        for (std::size_t row = 0; row < valid.size(); ++row) {
          const int c = valid[row];
          innovations_ << obs.gps_seconds << ',' << obs.gps_seconds << ',' << combinationMode_ << ",rtk_pv,mixed," << componentNames[c]
                       << ",-1,-1," << z[c] << ',' << z[c] - residual[static_cast<Eigen::Index>(row)] << ','
                       << residual[static_cast<Eigen::Index>(row)] << ',' << std::sqrt(s(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(row))) << ','
                       << normalized[static_cast<Eigen::Index>(row)] << ',' << nis << ',' << valid.size() << ",1\n";
        }
        error += k * residual;
        const Eigen::Matrix<double, 15, 15> identity = Eigen::Matrix<double, 15, 15>::Identity();
        const Eigen::Matrix<double, 15, 15> ikh = identity - k * h;
        p = ikh * p * ikh.transpose() + k * rv * k.transpose();

        state.ecef_m += rot.transpose() * error.segment<3>(0);
        state.lla_rad_m = navigation::ecefToLla(state.ecef_m);
        state.velocity_ned_mps += error.segment<3>(3);
        const Eigen::Vector3d attitudeCorrection = error.segment<3>(6);
        const double attitudeAngle = attitudeCorrection.norm();
        if (attitudeAngle > 1e-14) {
          const Eigen::Quaterniond correction(Eigen::AngleAxisd(attitudeAngle, attitudeCorrection / attitudeAngle));
          state.q_body_to_ned = (correction * state.q_body_to_ned).normalized();
        }
        const Eigen::Vector3d gyroBiasCorrection = error.segment<3>(9);
        const Eigen::Vector3d accelBiasCorrection = error.segment<3>(12);
        error.setZero();
        mechanizer.applyErrorStateCorrection(state, gyroBiasCorrection, accelBiasCorrection);
        ++updates;
      }

      // TC can run without RTK updates; seed its first reference state from the INS at the pair's previous epoch.
      if (tdDdEnabled_ && tdIndex < tdDd_.size() &&
          std::abs(tdDd_[tdIndex].previous_gps_seconds - sample.gps_seconds) <= 1e-4 &&
          (!previousEndpoint || std::abs(previousEndpoint->time - sample.gps_seconds) > 1e-4)) {
        previousEndpoint = Endpoint{sample.gps_seconds, state.ecef_m, state.velocity_ned_mps,
                                    p.block<3,3>(0,0), p.block<3,3>(3,3)};
      }

      bool endpointObserved = false;
      while (combinationMode_ == "tc" && tdDdEnabled_ && tdIndex < tdDd_.size() && tdDd_[tdIndex].gps_seconds <= sample.gps_seconds + 1e-6) {
        const auto& epoch = tdDd_[tdIndex++];
        if (std::abs(epoch.gps_seconds - sample.gps_seconds) > 1e-4 || epoch.observations.empty()) { ++tdRejected; continue; }
        if (!previousEndpoint || std::abs(previousEndpoint->time - epoch.previous_gps_seconds) > 1e-3) {
          ++tdRejected;
          endpointObserved = true;
          continue;
        }
        const Eigen::Vector3d currentEcef = state.ecef_m;
        const Eigen::Vector3d currentLla = navigation::ecefToLla(currentEcef);
        const Eigen::Matrix3d currentEcefToNed = ecefToNedRotation(currentLla);
        const Eigen::Matrix3d prevEcefToNed = ecefToNedRotation(navigation::ecefToLla(previousEndpoint->ecef));
        const Eigen::Matrix3d prevNedToEcef = prevEcefToNed.transpose();
        Eigen::VectorXd z(static_cast<Eigen::Index>(epoch.observations.size()));
        Eigen::VectorXd predicted(z.size());
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(z.size(), 15);
        Eigen::MatrixXd hPrev = Eigen::MatrixXd::Zero(z.size(), 3);
        for (Eigen::Index row = 0; row < z.size(); ++row) {
          const auto& o = epoch.observations[static_cast<std::size_t>(row)];
          auto range = [](const Eigen::Vector3d& receiver, const Eigen::Vector3d& satellite) { return (satellite - receiver).norm(); };
          const double geomCurrent = range(currentEcef, o.satellite_position_current_ecef_m) - range(base, o.satellite_position_current_ecef_m)
                                   - range(currentEcef, o.pivot_position_current_ecef_m) + range(base, o.pivot_position_current_ecef_m);
          const double geomPrevious = range(previousEndpoint->ecef, o.satellite_position_previous_ecef_m) - range(base, o.satellite_position_previous_ecef_m)
                                    - range(previousEndpoint->ecef, o.pivot_position_previous_ecef_m) + range(base, o.pivot_position_previous_ecef_m);
          predicted[row] = geomCurrent - geomPrevious;
          z[row] = o.delta_double_difference_m;
          auto grad = [](const Eigen::Vector3d& receiver, const Eigen::Vector3d& sat, const Eigen::Vector3d& pivot) {
            return (receiver - sat) / (receiver - sat).norm() - (receiver - pivot) / (receiver - pivot).norm();
          };
          h.block<1,3>(row, 0) = grad(currentEcef, o.satellite_position_current_ecef_m, o.pivot_position_current_ecef_m).transpose() * currentEcefToNed.transpose();
          hPrev.row(row) = -grad(previousEndpoint->ecef, o.satellite_position_previous_ecef_m, o.pivot_position_previous_ecef_m).transpose() * prevNedToEcef;
        }
        Eigen::MatrixXd r = epoch.covariance_m2 * tdDdScale_;
        if (r.rows() != z.size() || r.cols() != z.size()) { ++tdRejected; endpointObserved = true; continue; }
        // The prior endpoint is held as a nominal reference. Its position uncertainty is propagated into R;
        // cross-epoch state covariance is not retained by this first implementation.
        r += hPrev * previousEndpoint->p_position_ned * hPrev.transpose();
        r.diagonal().array() += minimumVariance_;
        const Eigen::VectorXd residual = z - predicted;
        const Eigen::MatrixXd s = h * p * h.transpose() + r;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(s);
        if (ldlt.info() != Eigen::Success) { ++tdRejected; endpointObserved = true; continue; }
        const Eigen::MatrixXd k = p * h.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(s.rows(), s.cols()));
        const double nis = residual.dot(ldlt.solve(residual));
        const Eigen::VectorXd normalized = residual.array() / s.diagonal().array().sqrt();
        for (Eigen::Index row = 0; row < z.size(); ++row) {
          const auto& o = epoch.observations[static_cast<std::size_t>(row)];
          innovations_ << epoch.gps_seconds << ',' << epoch.previous_gps_seconds << ',' << combinationMode_
                       << ",td_ddcp_range," << (o.band == CarrierPhaseBand::L1 ? "L1" : "L2") << ",range_m,"
                       << o.svid << ',' << o.pivot_svid << ',' << z[row] << ',' << predicted[row] << ',' << residual[row] << ','
                       << std::sqrt(s(row,row)) << ',' << normalized[row] << ',' << nis << ',' << z.size() << ",1\n";
        }
        error += k * residual;
        const Eigen::Matrix<double, 15, 15> identity = Eigen::Matrix<double, 15, 15>::Identity();
        const Eigen::MatrixXd ikh = identity - k * h;
        p = ikh * p * ikh.transpose() + k * r * k.transpose();
        state.ecef_m += currentEcefToNed.transpose() * error.segment<3>(0);
        state.lla_rad_m = navigation::ecefToLla(state.ecef_m);
        state.velocity_ned_mps += error.segment<3>(3);
        const Eigen::Vector3d da = error.segment<3>(6);
        if (da.norm() > 1e-14) state.q_body_to_ned = (Eigen::Quaterniond(Eigen::AngleAxisd(da.norm(), da.normalized())) * state.q_body_to_ned).normalized();
        const Eigen::Vector3d bg = error.segment<3>(9), ba = error.segment<3>(12);
        error.setZero();
        mechanizer.applyErrorStateCorrection(state, bg, ba);
        ++tdUpdates;
        tdRows += static_cast<std::size_t>(z.size());
        endpointObserved = true;
      }

      while (combinationMode_ == "ltc" && tdDdEnabled_ && tdIndex < tdDd_.size() && tdDd_[tdIndex].gps_seconds <= sample.gps_seconds + 1e-6) {
        const auto& epoch = tdDd_[tdIndex++];
        const double phaseDt = epoch.gps_seconds - epoch.previous_gps_seconds;
        if (std::abs(epoch.gps_seconds - sample.gps_seconds) > 1e-4 || !previousEndpoint ||
            std::abs(previousEndpoint->time - epoch.previous_gps_seconds) > 1e-3 || !(phaseDt > 0.0)) {
          ++tdRejected;
          endpointObserved = true;
          continue;
        }
        std::vector<Eigen::Index> l1Rows;
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(epoch.observations.size()); ++i)
          if (epoch.observations[static_cast<std::size_t>(i)].band == CarrierPhaseBand::L1) l1Rows.push_back(i);
        if (l1Rows.size() < 4 || epoch.covariance_m2.rows() != static_cast<Eigen::Index>(epoch.observations.size())) {
          ++tdRejected;
          endpointObserved = true;
          continue;
        }
        const Eigen::Index m = static_cast<Eigen::Index>(l1Rows.size());
        Eigen::MatrixXd design(m, 3), previousSensitivity(m, 3), phaseCovariance(m, m);
        Eigen::VectorXd observed(m), geometryAtZero(m);
        const Eigen::Vector3d r0 = previousEndpoint->ecef;
        auto range = [](const Eigen::Vector3d& receiver, const Eigen::Vector3d& satellite) { return (satellite - receiver).norm(); };
        auto ddGeometry = [&](const Eigen::Vector3d& receiver, const TdDdCarrierPhaseObservation& o, bool current) {
          const auto& sat = current ? o.satellite_position_current_ecef_m : o.satellite_position_previous_ecef_m;
          const auto& pivot = current ? o.pivot_position_current_ecef_m : o.pivot_position_previous_ecef_m;
          return range(receiver, sat) - range(base, sat) - range(receiver, pivot) + range(base, pivot);
        };
        for (Eigen::Index i = 0; i < m; ++i) {
          const Eigen::Index source = l1Rows[static_cast<std::size_t>(i)];
          const auto& o = epoch.observations[static_cast<std::size_t>(source)];
          observed[i] = o.delta_double_difference_m;
          geometryAtZero[i] = ddGeometry(r0, o, true) - ddGeometry(r0, o, false);
          design.row(i) = ((r0 - o.satellite_position_current_ecef_m) / range(r0, o.satellite_position_current_ecef_m)
                         - (r0 - o.pivot_position_current_ecef_m) / range(r0, o.pivot_position_current_ecef_m)).transpose();
          const Eigen::RowVector3d previousGeometryJacobian = ((r0 - o.satellite_position_previous_ecef_m) / range(r0, o.satellite_position_previous_ecef_m)
                         - (r0 - o.pivot_position_previous_ecef_m) / range(r0, o.pivot_position_previous_ecef_m)).transpose();
          previousSensitivity.row(i) = design.row(i) - previousGeometryJacobian;
          for (Eigen::Index j = 0; j < m; ++j)
            phaseCovariance(i, j) = epoch.covariance_m2(source, l1Rows[static_cast<std::size_t>(j)]) * tdDdScale_;
        }
        const Eigen::Matrix3d previousEcefToNed = ecefToNedRotation(navigation::ecefToLla(previousEndpoint->ecef));
        const Eigen::Matrix3d previousCovEcef = previousEcefToNed.transpose() * previousEndpoint->p_position_ned * previousEcefToNed;
        phaseCovariance += previousSensitivity * previousCovEcef * previousSensitivity.transpose();
        phaseCovariance.diagonal().array() += minimumVariance_;
        Eigen::LDLT<Eigen::MatrixXd> phaseFactor(phaseCovariance);
        if (phaseFactor.info() != Eigen::Success || !phaseFactor.isPositive()) {
          ++tdRejected;
          endpointObserved = true;
          continue;
        }
        const Eigen::Matrix3d information = design.transpose() * phaseFactor.solve(design);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spectrum(information);
        if (spectrum.info() != Eigen::Success || spectrum.eigenvalues()[0] <= spectrum.eigenvalues()[2] * 1e-8) {
          ++tdRejected;
          endpointObserved = true;
          continue;
        }
        Eigen::LDLT<Eigen::Matrix3d> informationFactor(information);
        if (informationFactor.info() != Eigen::Success || !informationFactor.isPositive()) {
          ++tdRejected;
          endpointObserved = true;
          continue;
        }
        const Eigen::Matrix3d deltaCovEcef = informationFactor.solve(Eigen::Matrix3d::Identity());
        const Eigen::Vector3d deltaEcef = deltaCovEcef * design.transpose() * phaseFactor.solve(observed - geometryAtZero);
        const Eigen::Matrix3d ecefToNed = ecefToNedRotation(state.lla_rad_m);
        const Eigen::Vector3d deltaVelocityNed = ecefToNed * deltaEcef / phaseDt;
        Eigen::Matrix3d velocityCovNed = ecefToNed * deltaCovEcef * ecefToNed.transpose() / (phaseDt * phaseDt);
        velocityCovNed += 0.25 * previousEndpoint->p_velocity_ned;
        Eigen::Matrix<double, 3, 15> h = Eigen::Matrix<double, 3, 15>::Zero();
        h.block<3,3>(0,3) = 0.5 * Eigen::Matrix3d::Identity();
        const Eigen::Vector3d predictedAverageVelocity = 0.5 * (previousEndpoint->velocity_ned + state.velocity_ned_mps);
        const Eigen::Vector3d residual = deltaVelocityNed - predictedAverageVelocity - 0.5 * error.segment<3>(3);
        const Eigen::Matrix3d s = h * p * h.transpose() + velocityCovNed;
        Eigen::LDLT<Eigen::Matrix3d> sFactor(s);
        if (sFactor.info() != Eigen::Success || !sFactor.isPositive()) {
          ++tdRejected;
          endpointObserved = true;
          continue;
        }
        const Eigen::Matrix<double, 15, 3> k = p * h.transpose() * sFactor.solve(Eigen::Matrix3d::Identity());
        const double nis = residual.dot(sFactor.solve(residual));
        const Eigen::Vector3d normalized = residual.cwiseQuotient(s.diagonal().cwiseSqrt());
        const char* components[] = {"velocity_n_mps", "velocity_e_mps", "velocity_d_mps"};
        for (int axis = 0; axis < 3; ++axis)
          innovations_ << epoch.gps_seconds << ',' << epoch.previous_gps_seconds << ",ltc,l1_tdcp_delta_position_velocity,L1,"
                       << components[axis] << ",-1," << (epoch.pivot_l1_svid) << ',' << deltaVelocityNed[axis] << ','
                       << state.velocity_ned_mps[axis] << ',' << residual[axis] << ',' << std::sqrt(s(axis,axis)) << ','
                       << normalized[axis] << ',' << nis << ",3,1\n";
        error += k * residual;
        const Eigen::Matrix<double, 15, 15> identity = Eigen::Matrix<double, 15, 15>::Identity();
        const Eigen::Matrix<double, 15, 15> ikh = identity - k * h;
        p = ikh * p * ikh.transpose() + k * velocityCovNed * k.transpose();
        state.ecef_m += ecefToNed.transpose() * error.segment<3>(0);
        state.lla_rad_m = navigation::ecefToLla(state.ecef_m);
        state.velocity_ned_mps += error.segment<3>(3);
        const Eigen::Vector3d da = error.segment<3>(6);
        if (da.norm() > 1e-14) state.q_body_to_ned = (Eigen::Quaterniond(Eigen::AngleAxisd(da.norm(), da.normalized())) * state.q_body_to_ned).normalized();
        const Eigen::Vector3d bg = error.segment<3>(9), ba = error.segment<3>(12);
        error.setZero();
        mechanizer.applyErrorStateCorrection(state, bg, ba);
        ++tdUpdates;
        tdRows += static_cast<std::size_t>(m);
        endpointObserved = true;
      }
      if (endpointObserved || (rtkIndex > 0 && std::abs(rtk_[rtkIndex - 1].gps_seconds - sample.gps_seconds) <= 1e-4)) {
        previousEndpoint = Endpoint{sample.gps_seconds, state.ecef_m, state.velocity_ned_mps,
                                    p.block<3,3>(0,0), p.block<3,3>(3,3)};
      }
      state.position_3d_rms_1sigma_m = std::sqrt(std::max(0.0, p.block<3, 3>(0, 0).trace()));
      state.velocity_3d_rms_1sigma_mps = std::sqrt(std::max(0.0, p.block<3, 3>(3, 3).trace()));
      state.attitude_3d_rms_1sigma_deg = std::sqrt(std::max(0.0, p.block<3, 3>(6, 6).trace())) * (180.0 / kPi);
      bus_->publish(outputTopic_, state);
    }
    std::cout << "[fusion:" << combinationMode_ << "] fused IMU samples=" << imu_.size() << ", RTK updates=" << updates
              << ", TD-DDCP epochs=" << tdUpdates << ", TD-DDCP rows=" << tdRows << ", TD rejected=" << tdRejected
              << ", final position sigma N/E/D=" << std::sqrt(std::max(0.0, p(0,0))) << '/'
              << std::sqrt(std::max(0.0, p(1,1))) << '/' << std::sqrt(std::max(0.0, p(2,2))) << " m" << std::endl;
  }

  MessageBus* bus_ = nullptr;
  std::string combinationMode_ = "ltc", imuTopic_, rtkTopic_, tdDdTopic_, outputTopic_, baseCoordinatesPath_;
  std::filesystem::path innovationPath_;
  navigation::InsInitialState initial_;
  Eigen::Vector3d initialPositionStd_ = Eigen::Vector3d::Constant(10.0);
  Eigen::Vector3d initialVelocityStd_ = Eigen::Vector3d::Constant(5.0);
  Eigen::Vector3d initialAttitudeStd_ = Eigen::Vector3d::Constant(10.0 * kPi / 180.0);
  Eigen::Vector3d initialGyroBiasStd_ = Eigen::Vector3d::Constant(0.01);
  Eigen::Vector3d initialAccelBiasStd_ = Eigen::Vector3d::Constant(0.1);
  std::vector<ImuSample> imu_;
  std::vector<RtkPositionVelocityObservation> rtk_;
  std::vector<PreparedTdDdCarrierPhaseEpoch> tdDd_;
  std::ofstream innovations_;
  bool enabled_ = true, tdDdEnabled_ = true, rtkPositionEnabled_ = true, rtkVelocityEnabled_ = true;
  bool acceptFloat_ = true, acceptFix_ = true, processed_ = false;
  double gyroNoiseDensity_ = 0.003, accelNoiseDensity_ = 0.05, gyroBiasRandomWalk_ = 0.0001, accelBiasRandomWalk_ = 0.001;
  double minimumVariance_ = 0.0004, floatScale_ = 1.0, fixScale_ = 1.0, tdDdScale_ = 1.0;
};

std::unique_ptr<IModule> makeLtcFusion() { return std::make_unique<LtcFusionModule>(); }
}

void registerLtcFusionModule(ModuleRegistry& registry) { registry.add("ltc_fusion", &makeLtcFusion); }
}
