#pragma once

#include "nav/gmis/gmis_lc_config.hpp"
#include "navcore/messages.hpp"
#include "navcore/state_estimator.hpp"

#include <map>
#include <iosfwd>
#include <vector>

namespace nav::gmis::tc {

struct Configuration {
  GmisLcConfiguration common;
  double c1_sigma_m = 0.5;
  double p2_sigma_m = 0.7;
  double l1_sigma_m = 0.012;
  double l2_sigma_m = 0.018;
  double ionosphere_initial_sigma_m = 10.0;
  double ambiguity_initial_sigma_cycles = 100.0;
  double ionosphere_rw_m_sqrt_s = 0.02;
  double ambiguity_rw_cycles_sqrt_s = 1e-4;
  double max_arc_gap_seconds = 2.0;
  double minimum_elevation_deg = 10.0;
  std::size_t minimum_common_satellites = 4;
  double nis_gate = 0.0; // 0 disables gating; all NIS values are still logged.

  static Configuration fromJson(const navcore::Json& json);
};

struct MeasurementRow {
  std::string type;
  int svid = 0;
  int pivot_svid = 0;
  int frequency = 0;
  double observed_m = 0.0;
  double predicted_m = 0.0;
  double innovation_m = 0.0;
};

class GmisTcEstimatorSession {
public:
  explicit GmisTcEstimatorSession(Configuration configuration);
  void initialize(double time_seconds, const Eigen::VectorXd& navigation_state,
                  const Eigen::MatrixXd& navigation_covariance);
  nav::core::EstimatorUpdateResult updateImu(const ImuSample& sample);
  nav::core::EstimatorUpdateResult updateGnss(const navcore::PreparedGnssEpoch& epoch);
  void writeAmbiguities(std::ostream& output, double gps_seconds) const;

  const Eigen::VectorXd& state() const noexcept { return estimator_.state(); }
  const Eigen::MatrixXd& covariance() const noexcept { return estimator_.covariance(); }
  const std::vector<MeasurementRow>& lastRows() const noexcept { return last_rows_; }
  int pivotSvid() const noexcept { return pivot_svid_; }
  std::size_t ambiguitySatelliteCount() const noexcept { return slots_.size(); }
  double timeSeconds() const noexcept { return time_seconds_; }

private:
  struct StateSlot { Eigen::Index ionosphere = -1, l1 = -1, l2 = -1; double last_seen = 0.0; };
  struct RawRow { std::string type; int svid=0; int frequency=0; double observed=0.0; double sigma=0.0; };

  void synchronizeStates(const navcore::PreparedGnssEpoch& epoch, int selected_pivot);
  std::vector<RawRow> makeRawRows(const navcore::PreparedGnssEpoch& epoch,
                                  int pivot, const Eigen::Vector3d& rover_ecef) const;
  Eigen::Vector3d antennaEcef(const Eigen::VectorXd& navigation_state) const;
  double geometricDoubleDifference(const Eigen::Vector3d& rover_ecef,
                                   const navcore::PreparedGnssEpoch& epoch,
                                   int svid, int pivot) const;

  Configuration config_;
  nav::core::StateEstimator estimator_;
  std::map<int, StateSlot> slots_;
  int pivot_svid_ = 0;
  double time_seconds_ = 0.0;
  bool initialized_ = false;
  std::vector<MeasurementRow> last_rows_;
};

} // namespace nav::gmis::tc
