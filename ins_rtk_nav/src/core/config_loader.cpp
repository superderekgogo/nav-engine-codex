#include "navcore/config_loader.hpp"
#include <fstream>
#include <sstream>

namespace navcore {
Json ConfigLoader::load(const std::filesystem::path& entry) const { return loadRecursive(std::filesystem::absolute(entry), 0); }
Json ConfigLoader::loadRecursive(const std::filesystem::path& file, unsigned depth) const {
  if (depth > 16) throw std::runtime_error("config include nesting exceeds 16: " + file.string());
  std::ifstream in(file, std::ios::binary); if (!in) throw std::runtime_error("cannot open config: " + file.string());
  std::ostringstream buffer; buffer << in.rdbuf(); Json current = Json::parse(buffer.str());
  if (!current.isObject()) throw std::runtime_error("config root must be a JSON object: " + file.string());
  Json merged(Json::Value(Json::Object{}));
  if (const Json* includes = current.find("include")) {
    if (!includes->isArray()) throw std::runtime_error("config include must be an array: " + file.string());
    for (const auto& include : includes->asArray()) {
      if (!include.isString()) throw std::runtime_error("config include entries must be strings: " + file.string());
      merged = mergeJson(merged, loadRecursive(file.parent_path() / include.asString(), depth + 1));
    }
  }
  Json::Object local;
  for (const auto& kv : current.asObject()) {
    if (kv.first == "include") continue;
    if (kv.first == "paths" && kv.second.isObject()) {
      Json::Object resolved;
      for (const auto& path : kv.second.asObject()) {
        if (path.second.isString()) {
          std::filesystem::path value(path.second.asString());
          if (value.is_relative()) value = (file.parent_path() / value).lexically_normal();
          resolved.emplace(path.first, Json(Json::Value(value.string())));
        } else resolved.emplace(path);
      }
      local.emplace(kv.first, Json(Json::Value(std::move(resolved))));
    } else local.emplace(kv);
  }
  return mergeJson(merged, Json(Json::Value(std::move(local))));
}
}
