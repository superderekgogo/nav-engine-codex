#pragma once
#include "navcore/json.hpp"
#include <filesystem>

namespace navcore {
class ConfigLoader {
public:
  Json load(const std::filesystem::path& entry) const;
private:
  Json loadRecursive(const std::filesystem::path& file, unsigned depth) const;
};
}
