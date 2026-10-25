#include <Eigen/Dense>

extern "C" {
#include "rtklib.h"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <vector>

namespace {
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
using Vector = Eigen::VectorXd;

struct AmbiguityPair {
  int pivot;
  int satellite;
  int system;
};

int ambiguityIndex(const rtk_t* rtk, int sat, int frequency) {
  return rtk->na + frequency * MAXSAT + sat;
}

bool eligible(const rtk_t* rtk, int sat) {
  const auto& status = rtk->ssat[sat];
  if (status.sys != SYS_GPS && status.sys != SYS_GLO && status.sys != SYS_GAL &&
      status.sys != SYS_CMP && status.sys != SYS_QZS && status.sys != SYS_SBS) return false;
  if (status.azel[1] < rtk->opt.elmaskar) return false;
  for (int frequency = 0; frequency < 2; ++frequency) {
    const int index = ambiguityIndex(rtk, sat, frequency);
    if (index < rtk->na || index >= rtk->nx || rtk->x[index] == 0.0 ||
        !status.vsat[frequency] || status.lock[frequency] < static_cast<unsigned>(rtk->opt.minlock) ||
        (status.slip[frequency] & 2)) return false;
  }
  return true;
}

void makeConstraintRows(const rtk_t* rtk, const std::vector<AmbiguityPair>& pairs,
                        bool wideLane, Matrix& H) {
  H = Matrix::Zero(static_cast<Eigen::Index>(pairs.size()), rtk->nx);
  for (std::size_t row = 0; row < pairs.size(); ++row) {
    const auto& pair = pairs[row];
    const int p1 = ambiguityIndex(rtk, pair.pivot, 0);
    const int s1 = ambiguityIndex(rtk, pair.satellite, 0);
    const int p2 = ambiguityIndex(rtk, pair.pivot, 1);
    const int s2 = ambiguityIndex(rtk, pair.satellite, 1);
    if (wideLane) {
      H(row, s1) = 1.0; H(row, p1) = -1.0;
      H(row, s2) = -1.0; H(row, p2) = 1.0;
    } else {
      H(row, s1) = 1.0; H(row, p1) = -1.0;
      H(row, s2) = 1.0; H(row, p2) = -1.0;
    }
  }
}

bool integerCandidates(const Vector& ambiguities, const Matrix& covariance,
                       std::vector<Vector>& candidates, std::vector<double>& costs) {
  constexpr int candidateCount = 32;
  const int n = static_cast<int>(ambiguities.size());
  if (n == 0 || covariance.rows() != n || covariance.cols() != n) return false;
  std::vector<double> values(static_cast<std::size_t>(n) * candidateCount);
  std::vector<double> scores(candidateCount);
  if (lambda(n, candidateCount, ambiguities.data(), covariance.data(), values.data(), scores.data()) != 0) return false;
  candidates.clear(); costs.clear();
  for (int candidate = 0; candidate < candidateCount; ++candidate) {
    Vector integers(n);
    for (int i = 0; i < n; ++i) {
      const double value = values[static_cast<std::size_t>(candidate) * n + i];
      if (!std::isfinite(value) || std::abs(value) > 1.0e9) return false;
      integers[i] = std::nearbyint(value);
    }
    candidates.push_back(std::move(integers));
    costs.push_back(scores[candidate]);
  }
  return true;
}

bool conditionOnIntegers(Vector& state, Matrix& covariance, const Matrix& H,
                         const Vector& integers) {
  const Matrix innovationCovariance = H * covariance * H.transpose();
  Eigen::LDLT<Matrix> factor(innovationCovariance);
  if (factor.info() != Eigen::Success || !factor.isPositive()) return false;
  const Matrix crossCovariance = covariance * H.transpose();
  const Matrix gain = factor.solve(crossCovariance.transpose()).transpose();
  if (factor.info() != Eigen::Success || !gain.allFinite()) return false;
  state += gain * (integers - H * state);
  covariance.noalias() -= gain * crossCovariance.transpose();
  covariance = 0.5 * (covariance + covariance.transpose()).eval();
  return state.allFinite() && covariance.allFinite();
}

bool resolveStage(const Matrix& H, const Vector& state, const Matrix& covariance,
                  const Vector* parityReference, double ratioThreshold,
                  Vector& fixedIntegers, double& ratio) {
  const Vector ambiguities = H * state;
  Matrix ambiguityCovariance = H * covariance * H.transpose();
  ambiguityCovariance = 0.5 * (ambiguityCovariance + ambiguityCovariance.transpose()).eval();
  std::vector<Vector> candidates;
  std::vector<double> costs;
  if (!integerCandidates(ambiguities, ambiguityCovariance, candidates, costs)) { ratio = 0.0; return false; }

  int best = -1, second = -1;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (parityReference) {
      bool parityMatches = true;
      for (Eigen::Index j = 0; j < candidates[i].size(); ++j) {
        const auto wl = static_cast<long long>(std::llround((*parityReference)[j]));
        const auto nl = static_cast<long long>(std::llround(candidates[i][j]));
        if (((wl - nl) & 1LL) != 0) { parityMatches = false; break; }
      }
      if (!parityMatches) continue;
    }
    if (best < 0) best = static_cast<int>(i);
    else { second = static_cast<int>(i); break; }
  }
  if (best < 0 || second < 0 || costs[best] <= 0.0) { ratio = 0.0; return false; }
  ratio = costs[second] / costs[best];
  if (!std::isfinite(ratio) || ratio < ratioThreshold) return false;
  fixedIntegers = candidates[best];
  return true;
}
} // namespace

extern "C" int resamb_WLNL(rtk_t* rtk, const obsd_t*, const int*, const int*, const int*,
                           int, const nav_t*, const double*) {
  static int diagnosticCount = 0;
  static int stageDiagnosticCount = 0;
  try {
    if (!rtk || rtk->opt.nf < 2 || rtk->na <= 0 || rtk->nx <= rtk->na) return 0;
    std::map<int, std::vector<int>> satellitesBySystem;
    for (int sat = 0; sat < MAXSAT; ++sat) if (eligible(rtk, sat))
      satellitesBySystem[rtk->ssat[sat].sys].push_back(sat);
    std::vector<AmbiguityPair> pairs;
    for (auto& [system, satellites] : satellitesBySystem) {
      if (satellites.size() < 2) continue;
      const auto pivot = std::max_element(satellites.begin(), satellites.end(), [&](int a, int b) {
        return rtk->ssat[a].azel[1] < rtk->ssat[b].azel[1];
      });
      const int pivotSat = *pivot;
      for (const int sat : satellites) if (sat != pivotSat) pairs.push_back({pivotSat, sat, system});
    }
    if (pairs.size() < 3) {
      if (diagnosticCount++ < 12) {
        int nonzero[2] = {}, dualValid = 0, dualLock = 0, dualNoSlip = 0, satStatus = 0;
        for (int sat = 0; sat < MAXSAT; ++sat) {
          const auto& s = rtk->ssat[sat];
          const bool x1 = rtk->x[ambiguityIndex(rtk, sat, 0)] != 0.0;
          const bool x2 = rtk->x[ambiguityIndex(rtk, sat, 1)] != 0.0;
          nonzero[0] += x1; nonzero[1] += x2;
          satStatus += s.sys != 0;
          dualValid += x1 && x2 && s.vsat[0] && s.vsat[1];
          dualLock += x1 && x2 && s.vsat[0] && s.vsat[1] && s.lock[0] >= static_cast<unsigned>(rtk->opt.minlock) && s.lock[1] >= static_cast<unsigned>(rtk->opt.minlock);
          dualNoSlip += x1 && x2 && s.vsat[0] && s.vsat[1] && !(s.slip[0] & 2) && !(s.slip[1] & 2);
        }
        std::fprintf(stderr, "[wlnl] insufficient DDs=%zu states=%d/%d dual=%d lock=%d no-slip=%d sat-status=%d minlock=%d\n", pairs.size(), nonzero[0], nonzero[1], dualValid, dualLock, dualNoSlip, satStatus, rtk->opt.minlock);
      }
      return 0;
    }

    Vector floatState = Eigen::Map<const Vector>(rtk->x, rtk->nx);
    Matrix floatCovariance = Eigen::Map<const Matrix>(rtk->P, rtk->nx, rtk->nx);
    if (!floatState.allFinite() || !floatCovariance.allFinite()) return 0;

    Matrix wideLaneH, narrowLaneH;
    makeConstraintRows(rtk, pairs, true, wideLaneH);
    makeConstraintRows(rtk, pairs, false, narrowLaneH);
    Vector wideLaneIntegers;
    double wideLaneRatio = 0.0;
    if (!resolveStage(wideLaneH, floatState, floatCovariance, nullptr,
                      rtk->opt.thresar[0], wideLaneIntegers, wideLaneRatio)) {
      if (diagnosticCount++ < 12) std::fprintf(stderr, "[wlnl] WL candidate rejected: dd=%zu ratio=%.3f\n", pairs.size(), wideLaneRatio);
      rtk->sol.ratio = static_cast<float>(wideLaneRatio);
      return 0;
    }
    if (stageDiagnosticCount < 20) std::fprintf(stderr, "[wlnl] WL accepted dd=%zu ratio=%.3f\n", pairs.size(), wideLaneRatio);

    Vector conditionedState = floatState;
    Matrix conditionedCovariance = floatCovariance;
    if (!conditionOnIntegers(conditionedState, conditionedCovariance, wideLaneH, wideLaneIntegers)) return 0;
    if (stageDiagnosticCount < 20) std::fprintf(stderr, "[wlnl] WL conditioned\n");

    Vector narrowLaneIntegers;
    double narrowLaneRatio = 0.0;
    if (!resolveStage(narrowLaneH, conditionedState, conditionedCovariance, &wideLaneIntegers,
                      rtk->opt.thresar[0], narrowLaneIntegers, narrowLaneRatio)) {
      if (diagnosticCount++ < 12) std::fprintf(stderr, "[wlnl] NL/parity candidate rejected after WL: dd=%zu WL=%.3f NL=%.3f\n", pairs.size(), wideLaneRatio, narrowLaneRatio);
      rtk->sol.ratio = static_cast<float>((std::min)(wideLaneRatio, narrowLaneRatio));
      return 0;
    }
    if (stageDiagnosticCount < 20) std::fprintf(stderr, "[wlnl] NL accepted ratio=%.3f\n", narrowLaneRatio);
    if (!conditionOnIntegers(conditionedState, conditionedCovariance, narrowLaneH, narrowLaneIntegers)) return 0;

    rtk->sol.ratio = static_cast<float>((std::min)(wideLaneRatio, narrowLaneRatio));
    for (int i = 0; i < rtk->na; ++i) rtk->xa[i] = conditionedState[i];
    for (int col = 0; col < rtk->na; ++col)
      for (int row = 0; row < rtk->na; ++row)
        rtk->Pa[row + col * rtk->na] = conditionedCovariance(row, col);
    if (stageDiagnosticCount++ < 20) std::fprintf(stderr, "[wlnl] FIX output ratio=%.3f\n", rtk->sol.ratio);
    return static_cast<int>(2 * pairs.size());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[wlnl] exception: %s\n", error.what());
    return 0;
  } catch (...) {
    std::fprintf(stderr, "[wlnl] unknown exception\n");
    return 0;
  }
}

extern "C" int resamb_TCAR(rtk_t*, const obsd_t*, const int*, const int*, const int*,
                            int, const nav_t*, const double*) {
  return 0;
}
