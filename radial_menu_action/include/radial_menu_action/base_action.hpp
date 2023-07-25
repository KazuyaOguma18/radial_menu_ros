#ifndef RADIAL_MENU_ACTION_BASE_ACTION_HPP
#define RADIAL_MENU_ACTION_BASE_ACTION_HPP

#include <memory>
#include <iostream>
#include <radial_menu_model/model.hpp>

namespace radial_menu_action {

template <typename T>
const std::vector< T > values(const std::string& values) {

  std::istringstream iss(values);
  std::vector< T > outputs;

  T v;
  while (iss >> v) {
    outputs.push_back(v);
  }
  return outputs;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
{
    os << "[ ";
    for (const auto& elem : vec) {
        os << elem << " ";
    }
    os << "]";
    return os;
}


class BaseAction;
typedef std::shared_ptr<BaseAction> BaseActionPtr;
typedef std::shared_ptr<const BaseAction> BaseActionConstPtr;

class BaseAction {
public:
  BaseAction(const radial_menu_model::ActionConstPtr &action) {
    action_.reset(new radial_menu_model::Action(*action));
  }

  virtual bool init() = 0;

  virtual void execute() const = 0;

  const int32_t id() const { return action_->id(); }

  bool get_attribute(const std::string& key, std::string& val) const {
    if (!action_->elm->getAttribute(key, &val)) {
      ROS_ERROR_STREAM("BaseAction::get_attribute(): No attribute '" << key << "'");
      return false;
    }
    return true;
  }

protected:
  radial_menu_model::ActionConstPtr action_;
};

} // namespace radial_menu_action

#endif 