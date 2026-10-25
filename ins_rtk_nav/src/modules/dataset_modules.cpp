#include "navcore/data_decode.hpp"
#include "navcore/module_manager.hpp"
#include "navcore/message_bus.hpp"
#include <iomanip>
#include <fstream>
#include <memory>
#include <sstream>

namespace navcore {
namespace {
Eigen::Vector3d readEcefCoordinate(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open base coordinate file: " + path);
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream row(line);
    Eigen::Vector3d xyz;
    if (row >> xyz.x() >> xyz.y() >> xyz.z()) {
      if (xyz.allFinite() && xyz.norm() > 1.0e6) return xyz;
    }
  }
  throw std::runtime_error("base coordinate file contains no valid ECEF XYZ triplet: " + path);
}
class DatasetReplay final : public IModule {
public:
  std::string_view type() const noexcept override { return "dataset_replay"; }
  Status configure(const Json& cfg) override {
    const auto& paths=cfg.at("paths");
    imuPath_=paths.at("imu").asString(); roverPath_=paths.at("rover_gnss").asString(); basePath_=paths.at("base_gnss").asString();
    baseCoordinatesPath_=paths.at("base_coordinates").asString();
    imuTopic_=cfg.at("imu_topic").asString(); gnssTopic_=cfg.at("gnss_topic").asString(); summaryTopic_=cfg.at("summary_topic").asString();
    const auto& gnss=cfg.at("gnss");
    tolerance_=gnss.at("epoch_tolerance_seconds").asNumber(); missingValue_=gnss.at("missing_value").asNumber();
    if (gnss.at("delimiter").asString() != "whitespace") return Status::failure("MVP GNSS decoder currently requires whitespace-delimited columns");
    return Status::success();
  }
  Status initialize(ModuleContext& ctx) override { bus_=&ctx.bus; return Status::success(); }
  Status start() override {
    try {
      ImuTextDecoder imuDecoder; GnssTextDecoder gnssDecoder(missingValue_); GnssPreprocessor preprocessor;
      const auto imu=imuDecoder.decodeFile(imuPath_);
      const auto rover=gnssDecoder.decodeFile(roverPath_);
      const auto base=gnssDecoder.decodeFile(basePath_);
      auto prepared=preprocessor.pair(rover,base,tolerance_);
      const auto basePosition = readEcefCoordinate(baseCoordinatesPath_);
      for (auto& epoch : prepared.epochs) epoch.base_station_position_ecef_m = basePosition;
      std::size_t imuIndex = 0, gnssIndex = 0;
      while (imuIndex < imu.size() || gnssIndex < prepared.epochs.size()) {
        const bool publishImu = gnssIndex >= prepared.epochs.size() ||
            (imuIndex < imu.size() && imu[imuIndex].gps_seconds <= prepared.epochs[gnssIndex].gps_seconds);
        if (publishImu) bus_->publish(imuTopic_, imu[imuIndex++]);
        else bus_->publish(gnssTopic_, prepared.epochs[gnssIndex++]);
      }
      prepared.summary.imu_samples=imu.size();
      bus_->publish(summaryTopic_,prepared.summary);
      bus_->publish("system.events", SystemEvent{"DatasetReplayComplete", "All time-ordered dataset records were published."});
      return Status::success();
    } catch (const std::exception& e) { return Status::failure(e.what()); }
  }
  void stop() noexcept override {}
private:
  MessageBus* bus_=nullptr;
  std::string imuPath_,roverPath_,basePath_,baseCoordinatesPath_,imuTopic_,gnssTopic_,summaryTopic_;
  double tolerance_=1e-6;
  double missingValue_=0.0;
};
std::unique_ptr<IModule> makeDatasetReplay(){return std::make_unique<DatasetReplay>();}
}
void registerDatasetModules(ModuleRegistry& r){r.add("dataset_replay",&makeDatasetReplay);}
}
