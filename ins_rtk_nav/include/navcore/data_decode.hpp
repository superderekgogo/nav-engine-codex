#pragma once
#include "navcore/messages.hpp"
#include <filesystem>
#include <vector>

namespace navcore {
class ImuTextDecoder {
public:
  std::vector<ImuSample> decodeFile(const std::filesystem::path& file) const;
};
class GnssTextDecoder {
public:
  explicit GnssTextDecoder(double missingValue = 0.0) : missingValue_(missingValue) {}
  std::vector<GnssObservation> decodeFile(const std::filesystem::path& file) const;
private:
  double missingValue_;
};
class GnssPreprocessor {
public:
  struct Result { std::vector<PreparedGnssEpoch> epochs; DatasetSummary summary; };
  Result pair(const std::vector<GnssObservation>& rover,
              const std::vector<GnssObservation>& base,
              double tolerance_seconds = 1e-6) const;
};
}
