#pragma once
#include <functional>
#include <map>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

namespace navcore {
class MessageBus {
  struct Slot { std::type_index type; std::function<void(const void*)> callback; };
public:
  template<class T, class F> void subscribe(const std::string& topic, F&& callback) {
    slots_[topic].push_back(Slot{std::type_index(typeid(T)), [fn = std::forward<F>(callback)](const void* p) { fn(*static_cast<const T*>(p)); }});
  }
  template<class T> void publish(const std::string& topic, const T& message) const {
    auto it = slots_.find(topic); if (it == slots_.end()) return;
    const std::type_index expected(typeid(T));
    for (const auto& slot : it->second) if (slot.type == expected) slot.callback(&message);
  }
private:
  std::map<std::string, std::vector<Slot>> slots_;
};
}
