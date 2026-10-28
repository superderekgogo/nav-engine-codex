#include "nav/gmis/gmis_lc_config.hpp"
#include "nav/gmis/gmis_lc_estimation.hpp"
#include "navcore/config_loader.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;

struct RtkRow {
  double time = 0.0;
  std::string status;
  int satellites = 0;
  int valid_components = 0;
  Eigen::Vector3d position_ecef = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ecef = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_variance = Eigen::Vector3d::Constant(1e6);
  Eigen::Vector3d velocity_variance = Eigen::Vector3d::Constant(1e6);
};

std::vector<std::string> split(const std::string& line, char delimiter) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string item;
  while (std::getline(stream, item, delimiter)) fields.push_back(item);
  return fields;
}

Eigen::Vector3d toNed(const Eigen::Vector3d& ecef, const nav::gmis::GeodeticOrigin& origin) {
  const double sl = std::sin(origin.latitude_rad), cl = std::cos(origin.latitude_rad);
  const double so = std::sin(origin.longitude_rad), co = std::cos(origin.longitude_rad);
  Eigen::Matrix3d c;
  c << -sl * co, -sl * so, cl,
       -so,       co,      0.0,
       -cl * co, -cl * so, -sl;
  const double e2 = kWgs84F * (2.0 - kWgs84F);
  const double rn = kWgs84A / std::sqrt(1.0 - e2 * sl * sl);
  const Eigen::Vector3d origin_ecef((rn + origin.height_m) * cl * co,
                                    (rn + origin.height_m) * cl * so,
                                    (rn * (1.0 - e2) + origin.height_m) * sl);
  return c * (ecef - origin_ecef);
}

Eigen::Vector3d ecefVectorToNed(const Eigen::Vector3d& vector,
                               const nav::gmis::GeodeticOrigin& origin) {
  const double sl = std::sin(origin.latitude_rad), cl = std::cos(origin.latitude_rad);
  const double so = std::sin(origin.longitude_rad), co = std::cos(origin.longitude_rad);
  Eigen::Matrix3d c;
  c << -sl * co, -sl * so, cl,
       -so,       co,      0.0,
       -cl * co, -cl * so, -sl;
  return c * vector;
}

Eigen::Vector3d nedToGeodetic(const Eigen::Vector3d& ned,
                              const nav::gmis::GeodeticOrigin& origin) {
  const double e2 = kWgs84F * (2.0 - kWgs84F);
  const double s = std::sin(origin.latitude_rad);
  const double den = std::sqrt(1.0 - e2 * s * s);
  const double rn = kWgs84A / den;
  const double rm = kWgs84A * (1.0 - e2) / (den * den * den);
  const double lat = origin.latitude_rad + ned.x() / (rm + origin.height_m);
  const double lon = origin.longitude_rad + ned.y() / ((rn + origin.height_m) * std::cos(origin.latitude_rad));
  const double height = origin.height_m - ned.z();
  return {lat * 180.0 / kPi, lon * 180.0 / kPi, height};
}

Eigen::Vector3d vector3(const navcore::Json& object, const char* key) {
  const auto& values = object.at(key).asArray();
  if (values.size() != 3) throw std::runtime_error(std::string("expected 3 values for ") + key);
  return {values[0].asNumber(), values[1].asNumber(), values[2].asNumber()};
}

std::filesystem::path resolve(const std::filesystem::path& config,
                              const std::string& path) {
  std::filesystem::path result(path);
  if (result.is_relative()) result = config.parent_path() / result;
  return std::filesystem::weakly_canonical(result);
}

std::vector<nav::gmis::ImuSample> readImu(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open IMU input: " + path.string());
  std::string line;
  std::getline(input, line); // documented header
  std::vector<nav::gmis::ImuSample> samples;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::istringstream row(line);
    nav::gmis::ImuSample sample;
    if (!(row >> sample.time_seconds >> sample.specific_force_mps2.x()
          >> sample.specific_force_mps2.y() >> sample.specific_force_mps2.z()
          >> sample.gyro_radps.x() >> sample.gyro_radps.y() >> sample.gyro_radps.z())) continue;
    sample.gyro_rate_convention = nav::gmis::ImuSample::GyroRateConvention::BodyRelativeInertial;
    samples.push_back(sample);
  }
  if (samples.empty()) throw std::runtime_error("IMU input has no samples");
  return samples;
}

std::vector<RtkRow> readRtk(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open RTK solution input: " + path.string());
  std::string line;
  std::getline(input, line); // header
  std::vector<RtkRow> rows;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto f = split(line, ',');
    if (f.size() < 17 || f[1] == "INVALID" || f[1] == "HOLD") continue;
    try {
      RtkRow r;
      r.time = std::stod(f[0]);
      r.status = f[1];
      r.satellites = std::stoi(f[2]);
      r.position_ecef = {std::stod(f[4]), std::stod(f[5]), std::stod(f[6])};
      r.velocity_ecef = {std::stod(f[7]), std::stod(f[8]), std::stod(f[9])};
      r.valid_components = std::stoi(f[10]);
      if ((r.valid_components & 0x07) == 0) continue;
      r.position_variance = {std::stod(f[11]), std::stod(f[12]), std::stod(f[13])};
      r.velocity_variance = {std::stod(f[14]), std::stod(f[15]), std::stod(f[16])};
      rows.push_back(r);
    } catch (const std::exception&) { /* skip malformed rows */ }
  }
  if (rows.empty()) throw std::runtime_error("RTK input has no valid rows");
  return rows;
}

Eigen::Matrix<double, 21, 21> initialCovariance(const navcore::Json& runtime) {
  Eigen::Matrix<double, 21, 21> p = Eigen::Matrix<double, 21, 21>::Zero();
  const auto pos = vector3(runtime, "initial_position_sigma_m");
  const auto vel = vector3(runtime, "initial_velocity_sigma_mps");
  const auto att = vector3(runtime, "initial_attitude_sigma_rad");
  const auto acc = vector3(runtime, "initial_acceleration_sigma_mps2");
  const auto rate = vector3(runtime, "initial_angular_rate_sigma_radps");
  p.diagonal().segment<3>(nav::gmis::StateIndex::position) = pos.array().square();
  p.diagonal().segment<3>(nav::gmis::StateIndex::velocity) = vel.array().square();
  p.diagonal().segment<3>(nav::gmis::StateIndex::acceleration) = acc.array().square();
  p.diagonal().segment<3>(nav::gmis::StateIndex::attitude) = att.array().square();
  p.diagonal().segment<3>(nav::gmis::StateIndex::angular_rate) = rate.array().square();
  const double sensor_sigma = runtime.at("initial_sensor_error_sigma").asNumber();
  p.diagonal().segment<6>(nav::gmis::StateIndex::gyro_bias).setConstant(sensor_sigma * sensor_sigma);
  return p;
}
} // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      std::cerr << "Usage: gmis_lc_app <gmis_lc.json>\n";
      return 2;
    }
    const auto config_path = std::filesystem::absolute(argv[1]);
    navcore::ConfigLoader loader;
    const auto json = loader.load(config_path);
    const auto model = nav::gmis::GmisLcConfiguration::fromJson(json);
    const auto& runtime = json.at("runtime");
    const auto imu_path = resolve(config_path, runtime.at("imu_file").asString());
    const auto rtk_path = resolve(config_path, runtime.at("rtk_solution_file").asString());
    const auto output_path = resolve(config_path, runtime.at("output_file").asString());
    const auto imu = readImu(imu_path);
    const auto rtk = readRtk(rtk_path);

    Eigen::VectorXd state = Eigen::VectorXd::Zero(nav::gmis::StateIndex::dimension);
    state.segment<3>(nav::gmis::StateIndex::attitude) = vector3(runtime, "initial_attitude_rph_rad");
    nav::gmis::GmisLcEstimatorSession session(model);
    session.initialize(imu.front().time_seconds, state, initialCovariance(runtime));

    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("cannot create GMIS output: " + output_path.string());
    const auto diagnostic_path = output_path.parent_path() / "rtk_update_diagnostics.csv";
    std::ofstream rtk_diagnostic(diagnostic_path);
    if (!rtk_diagnostic) throw std::runtime_error("cannot create RTK diagnostic output: " + diagnostic_path.string());
    output << "gps_seconds,latitude_deg,longitude_deg,height_m,north_m,east_m,down_m,vn_mps,ve_mps,vd_mps,roll_rad,pitch_rad,heading_rad,"
              "rms_n_m,rms_e_m,rms_d_m,rms_vn_mps,rms_ve_mps,rms_vd_mps,rms_roll_rad,rms_pitch_rad,rms_heading_rad,"
              "imu_update_accepted,imu_nis,rtk_update_accepted,rtk_nis,rtk_status\n";
    rtk_diagnostic << "gps_seconds,status,satellites,valid_components,"
                      "measurement_n_m,measurement_e_m,measurement_d_m,"
                      "predicted_n_m,predicted_e_m,predicted_d_m,"
                      "innovation_n_m,innovation_e_m,innovation_d_m,"
                      "post_n_m,post_e_m,post_d_m,post_residual_n_m,post_residual_e_m,post_residual_d_m,"
                      "correction_n_m,correction_e_m,correction_d_m,"
                      "configured_R_n_m2,configured_R_e_m2,configured_R_d_m2,"
                      "source_R_ecef_x_m2,source_R_ecef_y_m2,source_R_ecef_z_m2,"
                      "K_nn,K_ne,K_nd,K_en,K_ee,K_ed,K_dn,K_de,K_dd,"
                      "pre_sigma_n_m,pre_sigma_e_m,pre_sigma_d_m,post_sigma_n_m,post_sigma_e_m,post_sigma_d_m,"
                      "normalized_innovation_n,normalized_innovation_e,normalized_innovation_d,nis,accepted\n";
    std::size_t ri = 0;
    double last_imu_nis = 0.0, last_rtk_nis = 0.0;
    bool imu_accepted = true, rtk_accepted = true;
    std::size_t outputs = 0;
    double next_output_time = imu.front().time_seconds + 1.0;
    const auto emit = [&](double time, const std::string& status) {
      const auto& x = session.state();
      const auto& p = session.covariance();
      const auto lla = nedToGeodetic(x.segment<3>(nav::gmis::StateIndex::position), model.origin);
      output << std::setprecision(12) << time << ',' << lla.x() << ',' << lla.y() << ',' << lla.z();
      for (int i : {0, 1, 2, 3, 4, 5, 9, 10, 11}) output << ',' << x(i);
      for (int i : {0, 1, 2, 3, 4, 5, 9, 10, 11})
        output << ',' << std::sqrt(std::max(0.0, p(i, i)));
      output << ',' << (imu_accepted ? 1 : 0) << ',' << last_imu_nis << ','
             << (rtk_accepted ? 1 : 0) << ',' << last_rtk_nis << ',' << status << '\n';
      ++outputs;
    };

    const auto applyRtk = [&](const RtkRow& row) {
      const Eigen::Vector3d measured_position = toNed(row.position_ecef, model.origin);
      const Eigen::Vector3d measured_velocity = ecefVectorToNed(row.velocity_ecef, model.origin);
      auto measurement_noise = model.rtk_measurement_covariance;
      // RTKLIB marks weak or unavailable Doppler velocity with very large variances.
      for (int k = 0; k < 3; ++k)
        if (row.velocity_variance(k) > 10.0) measurement_noise(k + 3, k + 3) = 1e6;

      const double dt = row.time - session.timeSeconds();
      Eigen::VectorXd predicted_state = session.state();
      Eigen::MatrixXd predicted_covariance = session.covariance();
      if (dt > 0.0) {
        const auto prediction = nav::gmis::GmisLcModels::predict(session.state(), dt, model.process_noise);
        predicted_state = prediction.state;
        predicted_covariance = prediction.jacobian * session.covariance() * prediction.jacobian.transpose() +
                              prediction.process_noise;
      }
      const auto linearized = nav::gmis::GmisLcModels::linearizeRtkPositionVelocity(
          predicted_state, measured_position, measured_velocity, measurement_noise,
          model.antenna_lever_arm_body_m);
      const Eigen::MatrixXd innovation_covariance =
          linearized.jacobian * predicted_covariance * linearized.jacobian.transpose() + measurement_noise;
      const Eigen::MatrixXd gain = predicted_covariance * linearized.jacobian.transpose() *
          innovation_covariance.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

      const auto update = session.updateRtk(row.time, measured_position, measured_velocity, measurement_noise);
      const Eigen::Vector3d posterior_position = session.state().segment<3>(nav::gmis::StateIndex::position);
      const Eigen::Vector3d correction = posterior_position - predicted_state.segment<3>(nav::gmis::StateIndex::position);
      const Eigen::Vector3d post_residual = measured_position - posterior_position;
      rtk_diagnostic << std::setprecision(12) << row.time << ',' << row.status << ',' << row.satellites << ','
                     << row.valid_components;
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << measured_position(k);
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << predicted_state(nav::gmis::StateIndex::position + k);
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << update.innovation(k);
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << posterior_position(k);
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << post_residual(k);
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << correction(k);
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << measurement_noise(k, k);
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << row.position_variance(k);
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          rtk_diagnostic << ',' << gain(nav::gmis::StateIndex::position + i, j);
      for (int k = 0; k < 3; ++k)
        rtk_diagnostic << ',' << std::sqrt(std::max(0.0, predicted_covariance(nav::gmis::StateIndex::position + k,
                                                                             nav::gmis::StateIndex::position + k)));
      for (int k = 0; k < 3; ++k)
        rtk_diagnostic << ',' << std::sqrt(std::max(0.0, session.covariance()(nav::gmis::StateIndex::position + k,
                                                                            nav::gmis::StateIndex::position + k)));
      for (int k = 0; k < 3; ++k) rtk_diagnostic << ',' << update.normalized_innovation(k);
      rtk_diagnostic << ',' << update.nis << ',' << (update.accepted ? 1 : 0) << '\n';
      return update;
    };

    for (const auto& sample : imu) {
      while (ri < rtk.size() && rtk[ri].time <= sample.time_seconds) {
        if (rtk[ri].time >= session.timeSeconds()) {
          const auto result = applyRtk(rtk[ri]);
          rtk_accepted = result.accepted;
          last_rtk_nis = result.nis;
        }
        ++ri;
      }
      if (sample.time_seconds < session.timeSeconds()) continue;
      const auto result = session.updateImu(sample);
      imu_accepted = result.accepted;
      last_imu_nis = result.nis;
      if (sample.time_seconds + 1e-9 >= next_output_time) {
        emit(sample.time_seconds, "GMIS_LC");
        do { next_output_time += 1.0; }
        while (next_output_time <= sample.time_seconds + 1e-9);
      }
    }
    while (ri < rtk.size()) {
      if (rtk[ri].time >= session.timeSeconds()) {
        const auto result = applyRtk(rtk[ri]);
        rtk_accepted = result.accepted;
        last_rtk_nis = result.nis;
      }
      ++ri;
    }
    std::cout << "GMIS_LC replay complete: " << outputs << " output epochs\n"
              << "  IMU: " << imu.size() << " samples\n"
              << "  RTK: " << rtk.size() << " solutions\n"
              << "  RTK diagnostics: " << diagnostic_path.string() << "\n"
              << "  Output: " << output_path.string() << "\n"
              << "  Initial roll/pitch/heading are configuration seeds; this is a pipeline replay, not an accuracy validation.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "GMIS_LC failed: " << e.what() << '\n';
    return 1;
  }
}
