#include "navcore/data_decode.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace navcore {
namespace {
using Columns = std::unordered_map<std::string, std::size_t>;
std::vector<std::string> tokens(const std::string& line) {
  std::istringstream stream(line); std::vector<std::string> out; std::string token;
  while (stream >> token) out.push_back(std::move(token));
  return out;
}
Columns makeColumns(const std::vector<std::string>& names) {
  Columns columns; for (std::size_t i = 0; i < names.size(); ++i) columns.emplace(names[i], i); return columns;
}
std::size_t column(const Columns& columns, const std::string& name, const std::filesystem::path& file) {
  auto it = columns.find(name); if (it == columns.end()) throw std::runtime_error("missing column '" + name + "' in " + file.string()); return it->second;
}
double number(const std::string& text, const std::filesystem::path& file, std::size_t line) {
  std::size_t used = 0; double v;
  try { v = std::stod(text, &used); } catch (...) { throw std::runtime_error(file.string() + ":" + std::to_string(line) + ": invalid number '" + text + "'"); }
  if (used != text.size() || !std::isfinite(v)) throw std::runtime_error(file.string() + ":" + std::to_string(line) + ": invalid finite number '" + text + "'");
  return v;
}
std::optional<double> optionalMeasurement(const std::string& text, const std::filesystem::path& file, std::size_t line, double missingValue) {
  const double v = number(text, file, line); if (v == missingValue) return std::nullopt; return v;
}
std::string readHeader(std::ifstream& in, const std::filesystem::path& file) {
  std::string line; std::size_t lineNo = 0;
  while (std::getline(in, line)) { ++lineNo; if (!tokens(line).empty()) return line; }
  throw std::runtime_error("empty input file: " + file.string());
}
std::vector<std::string> recordTokens(const std::string& line, const std::filesystem::path& file, std::size_t lineNo, std::size_t expected) {
  auto fields = tokens(line); if (fields.empty()) return {};
  if (fields.size() < expected) throw std::runtime_error(file.string() + ":" + std::to_string(lineNo) + ": expected at least " + std::to_string(expected) + " columns, got " + std::to_string(fields.size()));
  return fields;
}
template<class T, class F> std::vector<T> decodeRows(const std::filesystem::path& file, F decode) {
  std::ifstream in(file); if (!in) throw std::runtime_error("cannot open data file: " + file.string());
  const auto names = tokens(readHeader(in, file)); const auto columns = makeColumns(names);
  std::vector<T> result; std::string line; std::size_t lineNo = 1; std::uint64_t sequence = 0;
  while (std::getline(in, line)) { ++lineNo; if (tokens(line).empty()) continue; auto fields = recordTokens(line, file, lineNo, names.size()); result.push_back(decode(columns, fields, lineNo, sequence++)); }
  return result;
}
std::optional<double> getOptional(const Columns& cols, const std::vector<std::string>& row, const std::string& name, const std::filesystem::path& file, std::size_t line, double missingValue) {
  auto it = cols.find(name); if (it == cols.end()) return std::nullopt;
  return optionalMeasurement(row.at(it->second), file, line, missingValue);
}
struct EpochGroup { double time = 0.0; std::size_t begin = 0, end = 0; };
std::vector<EpochGroup> groups(const std::vector<GnssObservation>& records, double tolerance) {
  std::vector<EpochGroup> result; std::size_t i = 0;
  while (i < records.size()) { const auto begin = i; const double t = records[i].gps_seconds; while (i < records.size() && std::abs(records[i].gps_seconds - t) <= tolerance) ++i; result.push_back({t, begin, i}); }
  return result;
}
}
std::vector<ImuSample> ImuTextDecoder::decodeFile(const std::filesystem::path& file) const {
  auto samples = decodeRows<ImuSample>(file, [&](const Columns& c, const std::vector<std::string>& row, std::size_t line, std::uint64_t seq) {
    ImuSample s; s.gps_seconds = number(row.at(column(c,"gpssec",file)),file,line);
    s.specific_force_mps2 = Eigen::Vector3d(number(row.at(column(c,"acc_x(m/s^2)",file)),file,line), number(row.at(column(c,"acc_y(m/s^2)",file)),file,line), number(row.at(column(c,"acc_z(m/s^2)",file)),file,line));
    s.gyro_radps = Eigen::Vector3d(number(row.at(column(c,"gyro_x(rad/s)",file)),file,line), number(row.at(column(c,"gyro_y(rad/s)",file)),file,line), number(row.at(column(c,"gyro_z(rad/s)",file)),file,line)); s.sequence = seq; return s;
  });
  for (std::size_t i = 1; i < samples.size(); ++i) if (samples[i].gps_seconds <= samples[i-1].gps_seconds) throw std::runtime_error("IMU timestamps are not strictly increasing near record " + std::to_string(i));
  return samples;
}
std::vector<GnssObservation> GnssTextDecoder::decodeFile(const std::filesystem::path& file) const {
  auto records = decodeRows<GnssObservation>(file, [&](const Columns& c, const std::vector<std::string>& row, std::size_t line, std::uint64_t seq) {
    GnssObservation o; o.gps_seconds = number(row.at(column(c,"gpssec",file)),file,line);
    const double id = number(row.at(column(c,"svid",file)),file,line); o.svid = static_cast<int>(id);
    if (o.svid <= 0 || static_cast<double>(o.svid) != id) throw std::runtime_error(file.string() + ":" + std::to_string(line) + ": invalid/missing SVID");
    o.c1_pseudorange_m = getOptional(c,row,"C1",file,line,missingValue_); o.p2_pseudorange_m = getOptional(c,row,"P2",file,line,missingValue_); o.l1_carrier_cycles = getOptional(c,row,"L1",file,line,missingValue_); o.d1_doppler_hz = getOptional(c,row,"D1",file,line,missingValue_); o.l2_carrier_cycles = getOptional(c,row,"L2",file,line,missingValue_);
    const auto x=getOptional(c,row,"SatX",file,line,missingValue_), y=getOptional(c,row,"SatY",file,line,missingValue_), z=getOptional(c,row,"SatZ",file,line,missingValue_);
    if (x && y && z) o.satellite_position_ecef_m = Eigen::Vector3d(*x,*y,*z);
    o.satellite_clock_m = getOptional(c,row,"SatClk",file,line,missingValue_); o.satellite_elevation_deg = getOptional(c,row,"SatElev",file,line,missingValue_); o.sequence=seq; return o;
  });
  std::stable_sort(records.begin(), records.end(), [](const auto& a,const auto& b){ return a.gps_seconds < b.gps_seconds || (a.gps_seconds == b.gps_seconds && a.svid < b.svid); });
  return records;
}
GnssPreprocessor::Result GnssPreprocessor::pair(const std::vector<GnssObservation>& roverInput, const std::vector<GnssObservation>& baseInput, double tolerance) const {
  if (!(tolerance >= 0.0) || !std::isfinite(tolerance)) throw std::runtime_error("invalid GNSS epoch tolerance");
  Result out; out.summary.rover_records=roverInput.size(); out.summary.base_records=baseInput.size();
  auto rover=groups(roverInput,tolerance), base=groups(baseInput,tolerance); std::size_t i=0,j=0;
  while (i<rover.size() && j<base.size()) {
    const double dt=rover[i].time-base[j].time;
    if (std::abs(dt)>tolerance) { if (dt<0) { ++out.summary.rover_epochs_without_base; ++i; } else { ++out.summary.base_epochs_without_rover; ++j; } continue; }
    std::map<int,const GnssObservation*> baseBySvid;
    for (std::size_t k=base[j].begin;k<base[j].end;++k) baseBySvid.emplace(baseInput[k].svid,&baseInput[k]);
    PreparedGnssEpoch epoch; epoch.gps_seconds=rover[i].time;
    for (std::size_t k=rover[i].begin;k<rover[i].end;++k) {
      const auto& r=roverInput[k]; auto found=baseBySvid.find(r.svid);
      if (found==baseBySvid.end()) { ++out.summary.satellites_without_base_match; continue; }
      const auto& b=*found->second;
      if (!b.satellite_position_ecef_m) { ++out.summary.satellites_without_position; continue; }
      PreparedSatellite s; s.svid=r.svid; s.rover_c1_m=r.c1_pseudorange_m; s.base_c1_m=b.c1_pseudorange_m; s.rover_p2_m=r.p2_pseudorange_m; s.base_p2_m=b.p2_pseudorange_m; s.rover_l1_cycles=r.l1_carrier_cycles; s.base_l1_cycles=b.l1_carrier_cycles; s.rover_d1_hz=r.d1_doppler_hz; s.base_d1_hz=b.d1_doppler_hz; s.rover_l2_cycles=r.l2_carrier_cycles; s.base_l2_cycles=b.l2_carrier_cycles; s.satellite_position_ecef_m=*b.satellite_position_ecef_m; s.satellite_clock_m=b.satellite_clock_m; s.satellite_elevation_deg=b.satellite_elevation_deg; epoch.satellites.push_back(std::move(s));
    }
    if (!epoch.satellites.empty()) { ++out.summary.matched_epochs; out.summary.matched_satellites+=epoch.satellites.size(); out.epochs.push_back(std::move(epoch)); }
    ++i; ++j;
  }
  out.summary.rover_epochs_without_base += rover.size()-i; out.summary.base_epochs_without_rover += base.size()-j;
  return out;
}
}
