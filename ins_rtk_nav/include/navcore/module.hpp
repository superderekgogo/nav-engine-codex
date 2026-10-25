#pragma once
#include "navcore/json.hpp"
#include "navcore/messages.hpp"
#include <memory>
#include <string>
#include <string_view>

namespace navcore {
class IModule {
public:
  virtual ~IModule() = default;
  virtual std::string_view type() const noexcept = 0;
  virtual Status configure(const Json& config) = 0;
  virtual Status initialize(ModuleContext& context) = 0;
  virtual Status start() = 0;
  virtual void stop() noexcept = 0;
};
using ModuleFactory = std::unique_ptr<IModule>(*)();
}
