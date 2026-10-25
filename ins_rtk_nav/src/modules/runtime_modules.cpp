#include "navcore/module_manager.hpp"
#include "navcore/message_bus.hpp"
#include "navcore/messages.hpp"
#include <iostream>

namespace navcore {
namespace {
class StatusSource final : public IModule {
public:
  std::string_view type() const noexcept override { return "status_source"; }
  Status configure(const Json& cfg) override {
    topic_ = cfg.at("publish_topic").asString(); detail_ = cfg.at("detail").asString();
    return Status::success();
  }
  Status initialize(ModuleContext& ctx) override { bus_ = &ctx.bus; return Status::success(); }
  Status start() override { bus_->publish(topic_, SystemEvent{"FrameworkStarted", detail_}); return Status::success(); }
  void stop() noexcept override {}
private:
  MessageBus* bus_ = nullptr;
  std::string topic_;
  std::string detail_;
};
class ConsoleSink final : public IModule {
public:
  std::string_view type() const noexcept override { return "console_sink"; }
  Status configure(const Json& cfg) override { topic_ = cfg.at("subscribe_topic").asString(); return Status::success(); }
  Status initialize(ModuleContext& ctx) override {
    ctx.bus.subscribe<SystemEvent>(topic_, [](const SystemEvent& e) { std::cout << "[event] " << e.name << ": " << e.detail << std::endl; });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override {}
private:
  std::string topic_;
};
class DatasetSummarySink final : public IModule {
public:
  std::string_view type() const noexcept override { return "dataset_summary_sink"; }
  Status configure(const Json& cfg) override { topic_ = cfg.at("subscribe_topic").asString(); return Status::success(); }
  Status initialize(ModuleContext& ctx) override {
    ctx.bus.subscribe<DatasetSummary>(topic_, [](const DatasetSummary& s) {
      std::cout << "[dataset] IMU=" << s.imu_samples << ", rover records=" << s.rover_records << ", base records=" << s.base_records << std::endl;
      std::cout << "[dataset] paired epochs=" << s.matched_epochs << ", paired satellites=" << s.matched_satellites << ", no base match=" << s.satellites_without_base_match << ", no base position=" << s.satellites_without_position << std::endl;
      std::cout << "[dataset] unmatched rover epochs=" << s.rover_epochs_without_base << ", unmatched base epochs=" << s.base_epochs_without_rover << std::endl;
    });
    return Status::success();
  }
  Status start() override { return Status::success(); }
  void stop() noexcept override {}
private:
  std::string topic_;
};
std::unique_ptr<IModule> makeStatusSource() { return std::make_unique<StatusSource>(); }
std::unique_ptr<IModule> makeConsoleSink() { return std::make_unique<ConsoleSink>(); }
std::unique_ptr<IModule> makeDatasetSummarySink() { return std::make_unique<DatasetSummarySink>(); }
}
void registerRuntimeModules(ModuleRegistry& r) { r.add("status_source", &makeStatusSource); r.add("console_sink", &makeConsoleSink); r.add("dataset_summary_sink", &makeDatasetSummarySink); }
}
