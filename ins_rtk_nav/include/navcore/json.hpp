#pragma once
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace navcore {
class Json {
public:
  using Array = std::vector<Json>;
  using Object = std::map<std::string, Json>;
  using Value = std::variant<std::nullptr_t, bool, double, std::string, Array, Object>;

  Json() : value_(nullptr) {}
  explicit Json(Value value) : value_(std::move(value)) {}
  static Json parse(const std::string& source);
  bool isNull() const;
  bool isBool() const;
  bool isNumber() const;
  bool isString() const;
  bool isArray() const;
  bool isObject() const;
  bool asBool() const;
  double asNumber() const;
  const std::string& asString() const;
  const Array& asArray() const;
  const Object& asObject() const;
  const Json* find(const std::string& key) const;
  const Json& at(const std::string& key) const;
  std::string stringOr(const std::string& fallback) const;
private:
  Value value_;
};
Json mergeJson(const Json& base, const Json& overlay);
}
