#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace navcore {
struct ImuSample {
  double gps_seconds = 0.0;
  Eigen::Vector3d gyro_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d specific_force_mps2 = Eigen::Vector3d::Zero();
  std::uint64_t sequence = 0;
};
struct GnssObservation {
  double gps_seconds = 0.0;
  int svid = 0;
  std::optional<double> c1_pseudorange_m;
  std::optional<double> p2_pseudorange_m;
  std::optional<double> l1_carrier_cycles;
  std::optional<double> d1_doppler_hz;
  std::optional<double> l2_carrier_cycles;
  std::optional<Eigen::Vector3d> satellite_position_ecef_m;
  std::optional<double> satellite_clock_m;
  std::optional<double> satellite_elevation_deg;
  std::uint64_t sequence = 0;
};
struct PreparedSatellite {
  int svid = 0;
  std::optional<double> rover_c1_m, base_c1_m;
  std::optional<double> rover_p2_m, base_p2_m;
  std::optional<double> rover_l1_cycles, base_l1_cycles;
  std::optional<double> rover_d1_hz, base_d1_hz;
  std::optional<double> rover_l2_cycles, base_l2_cycles;
  bool rover_l1_cycle_slip = false, base_l1_cycle_slip = false;
  bool rover_l2_cycle_slip = false, base_l2_cycle_slip = false;
  Eigen::Vector3d satellite_position_ecef_m = Eigen::Vector3d::Zero();
  std::optional<double> satellite_clock_m;
  std::optional<double> satellite_elevation_deg;
};
struct PreparedGnssEpoch {
  double gps_seconds = 0.0;
  // Known base antenna phase-center coordinate, ECEF meters (not satellite position).
  Eigen::Vector3d base_station_position_ecef_m = Eigen::Vector3d::Zero();
  std::vector<PreparedSatellite> satellites;
};
enum class CarrierPhaseBand { L1, L2 };
struct TdDdCarrierPhaseObservation {
  int svid = 0;
  int pivot_svid = 0;
  CarrierPhaseBand band = CarrierPhaseBand::L1;
  double wavelength_m = 0.0;
  double delta_double_difference_m = 0.0;
  bool cycle_slip_screened = false;
  bool cycle_slip_detected = false;
  double doppler_phase_residual_cycles = 0.0;
  Eigen::Vector3d satellite_position_previous_ecef_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d satellite_position_current_ecef_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d pivot_position_previous_ecef_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d pivot_position_current_ecef_m = Eigen::Vector3d::Zero();
  std::optional<double> elevation_previous_deg;
  std::optional<double> elevation_current_deg;
};
struct PreparedTdDdCarrierPhaseEpoch {
  double previous_gps_seconds = 0.0;
  double gps_seconds = 0.0;
  std::uint64_t sequence = 0;
  int pivot_l1_svid = 0;
  int pivot_l2_svid = 0;
  std::vector<TdDdCarrierPhaseObservation> observations;
  Eigen::MatrixXd covariance_m2;
  bool reference_epoch_valid = false;
  bool pivot_changed = false;
  std::string status;
};
struct TdDdBuildSummary {
  std::uint64_t input_epochs = 0;
  std::uint64_t output_epochs = 0;
  std::uint64_t output_observations = 0;
  std::uint64_t cycle_slip_screened_observations = 0;
  std::uint64_t cycle_slip_rejected_observations = 0;
  std::uint64_t priming_epochs = 0;
  std::uint64_t rejected_gap_pairs = 0;
  std::uint64_t pivot_reset_pairs = 0;
  std::uint64_t epochs_without_valid_phase = 0;
};
struct RtkPositionVelocityObservation {
  double gps_seconds = 0.0;
  Eigen::Vector3d antenna_position_ecef_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d antenna_velocity_ecef_mps = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  std::uint8_t valid_components = 0; // bits 0..2 position XYZ, 3..5 velocity XYZ
  std::string solution_status;
  int used_satellites = 0;
  double ambiguity_ratio = 0.0;
};
struct DynamicStatus {
  double gps_seconds = 0.0;
  std::uint64_t epoch_id = 0;
  std::string state = "unknown"; // stationary, moving, or unknown
  double confidence = 0.0;
  double gyro_rms_radps = 0.0;
  double specific_force_norm_std_mps2 = 0.0;
  double speed_mps = 0.0;
  bool high_dynamic = false;
};
struct ConstraintObservation {
  double gps_seconds = 0.0;
  std::uint64_t epoch_id = 0;
  std::string type; // zupt, nhc, zpc, or zhc
  double confidence = 0.0;
};
struct NavigationSolution {
  double gps_seconds = 0.0;
  std::uint64_t epoch_id = 0;
  Eigen::Vector3d lla_rad_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d ecef_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ned_mps = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_body_to_ned = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyro_bias_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d lever_arm_body_m = Eigen::Vector3d::Zero();
  std::optional<double> position_3d_rms_1sigma_m;
  std::optional<double> velocity_3d_rms_1sigma_mps;
  std::optional<double> attitude_3d_rms_1sigma_deg;
  std::optional<double> roll_rms_1sigma_deg;
  std::optional<double> pitch_rms_1sigma_deg;
  std::optional<double> heading_rms_1sigma_deg;
  std::optional<double> free_heading_rms_1sigma_deg;
  bool heading_lock_active = false;
  std::uint64_t heading_lock_id = 0;
  std::optional<double> heading_lock_heading_deg;
  std::optional<double> heading_lock_applied_correction_deg;
};
struct InertialStateSnapshot {
  double gps_seconds = 0.0;
  std::uint64_t epoch_id = 0;
  ImuSample imu;
  NavigationSolution predicted_state;
};
struct ReplayControl { bool reset = false; bool complete = false; };
struct NavigationMeasurementBundle {
  double gps_seconds = 0.0;
  std::uint64_t epoch_id = 0;
  std::optional<RtkPositionVelocityObservation> rtk;
  std::optional<PreparedTdDdCarrierPhaseEpoch> td_ddcp;
  std::vector<ConstraintObservation> constraints;
};
struct ErrorStateFeedback {
  double gps_seconds = 0.0;
  std::uint64_t epoch_id = 0;
  Eigen::VectorXd correction;
};
struct FeedbackApplied { double gps_seconds = 0.0; std::uint64_t epoch_id = 0; bool accepted = false; };
struct HeadingLockControl {
  bool active = false;
  std::uint64_t lock_id = 0;
  double heading_rad = 0.0;
  double sigma_rad = 0.0;
  Eigen::Vector3d reference_lla_rad_m = Eigen::Vector3d::Zero();
};
struct EstimatorOutput {
  double gps_seconds = 0.0;
  std::uint64_t epoch_id = 0;
  double position_3d_rms_1sigma_m = 0.0;
  double velocity_3d_rms_1sigma_mps = 0.0;
  double attitude_3d_rms_1sigma_deg = 0.0;
  double roll_rms_1sigma_deg = 0.0;
  double pitch_rms_1sigma_deg = 0.0;
  double heading_rms_1sigma_deg = 0.0;
  double free_heading_rms_1sigma_deg = 0.0;
  bool heading_lock_active = false;
  std::uint64_t heading_lock_id = 0;
  double heading_lock_heading_deg = 0.0;
  double heading_lock_applied_correction_deg = 0.0;
};
struct DatasetSummary {
  std::uint64_t imu_samples = 0;
  std::uint64_t rover_records = 0;
  std::uint64_t base_records = 0;
  std::uint64_t matched_epochs = 0;
  std::uint64_t matched_satellites = 0;
  std::uint64_t rover_epochs_without_base = 0;
  std::uint64_t base_epochs_without_rover = 0;
  std::uint64_t satellites_without_base_match = 0;
  std::uint64_t satellites_without_position = 0;
};
struct SystemEvent { std::string name; std::string detail; };
struct ModuleContext { class MessageBus& bus; };
struct Status { bool ok = true; std::string message; static Status success() { return {}; } static Status failure(std::string m) { return {false, std::move(m)}; } };
}
