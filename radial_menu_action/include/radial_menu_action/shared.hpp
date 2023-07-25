#ifndef RADIAL_MENU_ACTION_SHARED_HPP
#define RADIAL_MENU_ACTION_SHARED_HPP

#include <map>
#include <mutex>
#include <memory>

namespace radial_menu_action {

class Shared {
public:
  Shared() : shared_(std::make_shared<Shared>()) {}

  void add(const std::string& key, const std::string& val) {
    std::lock_guard<std::mutex> lock(mtx_);
    shared_->mapstr_[key] = val;
  }

  void remove(const std::string& key) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (shared_->mapstr_.find(key) != shared_->mapstr_.end()) {
      shared_->mapstr_.erase(key);
    }
  }

  std::string get_required(const std::string& key) const {
    return shared_->mapstr_.at(key);
  }

  std::string get(const std::string& key, std::string &default_val) const {
    if (shared_->mapstr_.find(key) == shared_->mapstr_.end()) {
      return default_val;
    }
    return shared_->mapstr_.at(key);
  }

  void set(const std::string& key, const std::string& val) {
    this->add(key, val);
  }

  bool has(const std::string& key) const {
    return shared_->mapstr_.find(key) != shared_->mapstr_.end();
  }

private:
  const std::shared_ptr<Shared> shared_;
  std::map<std::string, std::string> mapstr_;
  std::mutex mtx_;
};
} // namespace radial_menu_action


#endif