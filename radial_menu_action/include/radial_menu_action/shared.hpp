#ifndef RADIAL_MENU_ACTION_SHARED_HPP
#define RADIAL_MENU_ACTION_SHARED_HPP

#include <map>
#include <mutex>
#include <memory>

namespace radial_menu_action {

class Shared;
typedef std::shared_ptr<Shared> SharedPtr;

class Shared {
public:
  Shared() {}

  static void add(const std::string& key, const std::string& val) {
    std::lock_guard<std::mutex> lock(mtx_);
    mapstr_[key] = val;
  }

  static void remove(const std::string& key) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (mapstr_.find(key) != mapstr_.end()) {
      mapstr_.erase(key);
    }
  }

  static std::string get_required(const std::string& key) {
    return mapstr_.at(key);
  }

  static std::string get(const std::string& key, const std::string& default_val = "") {
    if (mapstr_.find(key) == mapstr_.end()) {
      return default_val;
    }
    return mapstr_.at(key);
  }

  static void set(const std::string& key, const std::string& val) {
    add(key, val);
  }

  static bool has(const std::string& key) {
    return mapstr_.find(key) != mapstr_.end();
  }

private:
  static std::map<std::string, std::string> mapstr_;
  static std::mutex mtx_;
};

std::map<std::string, std::string> Shared::mapstr_ = {};
std::mutex Shared::mtx_;

} // namespace radial_menu_action


#endif