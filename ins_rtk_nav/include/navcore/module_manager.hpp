#pragma once
#include "navcore/module.hpp"
#include "navcore/message_bus.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace navcore {
class ModuleRegistry {
public:
  void add(std::string type, ModuleFactory factory);
  std::unique_ptr<IModule> create(const std::string& type) const;
private:
  std::map<std::string, ModuleFactory> factories_;
};
class ModuleManager {
public:
  explicit ModuleManager(MessageBus& bus) : bus_(bus) {}
  Status configureAndStart(const Json& config, const ModuleRegistry& registry);
  void stop() noexcept;
private:
  MessageBus& bus_;
  std::vector<std::unique_ptr<IModule>> modules_;
};
void registerRuntimeModules(ModuleRegistry& registry);
void registerDatasetModules(ModuleRegistry& registry);
void registerNavigatorModule(ModuleRegistry& registry);
void registerLtcFusionModule(ModuleRegistry& registry);
void registerMeasurementModules(ModuleRegistry& registry);
void registerGnssPreprocessingModules(ModuleRegistry& registry);
void registerRtkModules(ModuleRegistry& registry);
}
