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

  void add(const std::string& key, const std::string& val) {
    std::lock_guard<std::mutex> lock(mtx_);
    mapstr_[key] = val;
  }

  void remove(const std::string& key) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (mapstr_.find(key) != mapstr_.end()) {
      mapstr_.erase(key);
    }
  }

  std::string get_required(const std::string& key) const {
    return mapstr_.at(key);
  }

  std::string get(const std::string& key, const std::string& default_val = "") const {
    if (mapstr_.find(key) == mapstr_.end()) {
      return default_val;
    }
    return mapstr_.at(key);
  }

  void set(const std::string& key, const std::string& val) {
    this->add(key, val);
  }

  bool has(const std::string& key) const {
    return mapstr_.find(key) != mapstr_.end();
  }

private:
  std::map<std::string, std::string> mapstr_;
  std::mutex mtx_;
};
} // namespace radial_menu_action


#endif