#include "navcore/config_loader.hpp"
#include "navcore/module_manager.hpp"
#include "navcore/message_bus.hpp"
#include <iostream>

int main(int argc, char** argv) {
  try {
    if (argc != 2) { std::cerr << "Usage: ins_rtk_app <config.json>" << std::endl; return 2; }
    navcore::ConfigLoader loader;
    const auto config = loader.load(argv[1]);
    const auto appName = config.at("runtime").at("app_name").asString();
    navcore::MessageBus bus;
    navcore::ModuleRegistry registry;
    navcore::registerRuntimeModules(registry);
    navcore::registerDatasetModules(registry);
    navcore::registerNavigatorModule(registry);
  navcore::registerLtcFusionModule(registry);
    navcore::registerMeasurementModules(registry);
    navcore::registerGnssPreprocessingModules(registry);
    navcore::registerRtkModules(registry);
    navcore::ModuleManager manager(bus);
    const auto status = manager.configureAndStart(config, registry);
    if (!status.ok) { std::cerr << "Startup failed: " << status.message << std::endl; manager.stop(); return 1; }
    std::cout << appName << " is running with " << config.at("modules").asArray().size() << " modules." << std::endl;
    manager.stop();
    return 0;
  } catch (const std::exception& e) { std::cerr << "Fatal: " << e.what() << std::endl; return 1; }
}
