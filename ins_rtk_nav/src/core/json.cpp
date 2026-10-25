#include "navcore/json.hpp"
#include <cctype>
#include <cstdlib>
#include <sstream>

namespace navcore {
namespace {
class Parser {
public:
  explicit Parser(const std::string& text) : text_(text) {}
  Json run() { skip(); Json v = value(); skip(); if (pos_ != text_.size()) fail("trailing characters"); return v; }
private:
  const std::string& text_; size_t pos_ = 0;
  [[noreturn]] void fail(const std::string& why) const { throw std::runtime_error("JSON parse error at byte " + std::to_string(pos_) + ": " + why); }
  void skip() { while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
  bool take(char c) { skip(); if (pos_ < text_.size() && text_[pos_] == c) { ++pos_; return true; } return false; }
  void expect(char c) { if (!take(c)) fail(std::string("expected '") + c + "'"); }
  Json value() {
    skip(); if (pos_ >= text_.size()) fail("unexpected end");
    char c = text_[pos_];
    if (c == '{') return object(); if (c == '[') return array(); if (c == '"') return Json(Json::Value(string()));
    if (c == 't') { literal("true"); return Json(Json::Value(true)); }
    if (c == 'f') { literal("false"); return Json(Json::Value(false)); }
    if (c == 'n') { literal("null"); return Json(); }
    if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return number();
    fail("unexpected token");
  }
  void literal(const char* word) { while (*word) { if (pos_ >= text_.size() || text_[pos_++] != *word++) fail("invalid literal"); } }
  unsigned hex4() {
    if (pos_ + 4 > text_.size()) fail("incomplete Unicode escape");
    unsigned code = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = text_[pos_++];
      code <<= 4;
      if (c >= '0' && c <= '9') code += static_cast<unsigned>(c - '0');
      else if (c >= 'a' && c <= 'f') code += static_cast<unsigned>(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F') code += static_cast<unsigned>(c - 'A' + 10);
      else fail("invalid Unicode escape");
    }
    return code;
  }
  void appendUtf8(std::string& out, unsigned code) {
    if (code <= 0x7f) out.push_back(static_cast<char>(code));
    else if (code <= 0x7ff) { out.push_back(static_cast<char>(0xc0 | (code >> 6))); out.push_back(static_cast<char>(0x80 | (code & 0x3f))); }
    else if (code <= 0xffff) { out.push_back(static_cast<char>(0xe0 | (code >> 12))); out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3f))); out.push_back(static_cast<char>(0x80 | (code & 0x3f))); }
    else { out.push_back(static_cast<char>(0xf0 | (code >> 18))); out.push_back(static_cast<char>(0x80 | ((code >> 12) & 0x3f))); out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3f))); out.push_back(static_cast<char>(0x80 | (code & 0x3f))); }
  }
  std::string string() {
    expect('"'); std::string out;
    while (pos_ < text_.size()) {
      char c = text_[pos_++]; if (c == '"') return out;
      if (static_cast<unsigned char>(c) < 0x20) fail("control character in string");
      if (c != '\\') { out.push_back(c); continue; }
      if (pos_ >= text_.size()) fail("incomplete escape");
      char e = text_[pos_++];
      switch (e) {
        case '"': out.push_back('"'); break; case '\\': out.push_back('\\'); break; case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break; case 'f': out.push_back('\f'); break; case 'n': out.push_back('\n'); break; case 'r': out.push_back('\r'); break; case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned code = hex4();
          if (code >= 0xd800 && code <= 0xdbff) {
            if (pos_ + 2 > text_.size() || text_[pos_] != '\\' || text_[pos_ + 1] != 'u') fail("high surrogate without low surrogate");
            pos_ += 2; const unsigned low = hex4();
            if (low < 0xdc00 || low > 0xdfff) fail("invalid low surrogate");
            code = 0x10000 + ((code - 0xd800) << 10) + (low - 0xdc00);
          } else if (code >= 0xdc00 && code <= 0xdfff) fail("unexpected low surrogate");
          appendUtf8(out, code); break;
        }
        default: fail("invalid string escape");
      }
    }
    fail("unterminated string");
  }
  Json number() {
    size_t start = pos_; if (text_[pos_] == '-') ++pos_;
    if (pos_ < text_.size() && text_[pos_] == '0') ++pos_; else { if (pos_ >= text_.size() || !std::isdigit(static_cast<unsigned char>(text_[pos_]))) fail("invalid number"); while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
    if (pos_ < text_.size() && text_[pos_] == '.') { ++pos_; if (pos_ >= text_.size() || !std::isdigit(static_cast<unsigned char>(text_[pos_]))) fail("invalid fraction"); while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
    if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) { ++pos_; if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) ++pos_; if (pos_ >= text_.size() || !std::isdigit(static_cast<unsigned char>(text_[pos_]))) fail("invalid exponent"); while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
    return Json(Json::Value(std::strtod(text_.substr(start, pos_ - start).c_str(), nullptr)));
  }
  Json object() {
    expect('{'); Json::Object out; if (take('}')) return Json(Json::Value(std::move(out)));
    do { skip(); if (pos_ >= text_.size() || text_[pos_] != '"') fail("object key must be a string"); auto key = string(); expect(':'); auto inserted = out.emplace(key, value()); if (!inserted.second) fail("duplicate object key: " + key); } while (take(',')); expect('}'); return Json(Json::Value(std::move(out)));
  }
  Json array() { expect('['); Json::Array out; if (take(']')) return Json(Json::Value(std::move(out))); do { out.push_back(value()); } while (take(',')); expect(']'); return Json(Json::Value(std::move(out))); }
};
}
Json Json::parse(const std::string& s) { return Parser(s).run(); }
bool Json::isNull() const { return std::holds_alternative<std::nullptr_t>(value_); }
bool Json::isBool() const { return std::holds_alternative<bool>(value_); }
bool Json::isNumber() const { return std::holds_alternative<double>(value_); }
bool Json::isString() const { return std::holds_alternative<std::string>(value_); }
bool Json::isArray() const { return std::holds_alternative<Array>(value_); }
bool Json::isObject() const { return std::holds_alternative<Object>(value_); }
bool Json::asBool() const { return std::get<bool>(value_); }
double Json::asNumber() const { return std::get<double>(value_); }
const std::string& Json::asString() const { return std::get<std::string>(value_); }
const Json::Array& Json::asArray() const { return std::get<Array>(value_); }
const Json::Object& Json::asObject() const { return std::get<Object>(value_); }
const Json* Json::find(const std::string& k) const { if (!isObject()) return nullptr; auto it = asObject().find(k); return it == asObject().end() ? nullptr : &it->second; }
const Json& Json::at(const std::string& k) const { auto p = find(k); if (!p) throw std::runtime_error("missing JSON key: " + k); return *p; }
std::string Json::stringOr(const std::string& f) const { return isString() ? asString() : f; }
Json mergeJson(const Json& a, const Json& b) {
  if (!a.isObject() || !b.isObject()) return b;
  Json::Object merged = a.asObject();
  for (const auto& kv : b.asObject()) { auto it = merged.find(kv.first); merged[kv.first] = it == merged.end() ? kv.second : mergeJson(it->second, kv.second); }
  return Json(Json::Value(std::move(merged)));
}
}
