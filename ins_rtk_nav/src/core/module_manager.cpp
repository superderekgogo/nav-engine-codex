#include "navcore/module_manager.hpp"
#include <stdexcept>

namespace navcore {
void ModuleRegistry::add(std::string type, ModuleFactory factory) {
  if (!factory || !factories_.emplace(std::move(type), factory).second) throw std::runtime_error("duplicate or invalid module registration");
}
std::unique_ptr<IModule> ModuleRegistry::create(const std::string& type) const {
  auto it = factories_.find(type); if (it == factories_.end()) throw std::runtime_error("unknown module type: " + type); return it->second();
}
Status ModuleManager::configureAndStart(const Json& config, const ModuleRegistry& registry) {
  try {
    const auto& definitions = config.at("modules").asArray();
    for (const auto& def : definitions) {
      auto module = registry.create(def.at("type").asString());
      Json::Object merged = def.asObject();
      if (const Json* paths = config.find("paths")) merged["paths"] = *paths;
      if (const Json* gnss = config.find("gnss")) merged["gnss"] = *gnss;
      if (const Json* ins = config.find("ins")) merged["ins"] = *ins;
      if (const Json* fusionModel = config.find("fusion_model")) merged["fusion_model"] = *fusionModel;
      if (const Json* rtkSolution = config.find("rtk_solution")) merged["rtk_solution"] = *rtkSolution;
      if (const Json* cycleSlip = config.find("cycle_slip_preprocessing")) merged["cycle_slip_preprocessing"] = *cycleSlip;
      Json moduleConfig(Json::Value(std::move(merged)));
      auto status = module->configure(moduleConfig); if (!status.ok) return status;
      modules_.push_back(std::move(module));
    }
    ModuleContext context{bus_};
    for (auto& module : modules_) { auto status = module->initialize(context); if (!status.ok) return status; }
    for (auto& module : modules_) { auto status = module->start(); if (!status.ok) return status; }
    return Status::success();
  } catch (const std::exception& e) { return Status::failure(e.what()); }
}
void ModuleManager::stop() noexcept { for (auto it = modules_.rbegin(); it != modules_.rend(); ++it) (*it)->stop(); modules_.clear(); }
}
