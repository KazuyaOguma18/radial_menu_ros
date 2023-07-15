#ifndef RADIAL_MENU_ACTION_BASE_ACTION_HPP
#define RADIAL_MENU_ACTION_BASE_ACTION_HPP

#include <memory>
#include <radial_menu_model/model.hpp>

namespace radial_menu_action {

class BaseAction;
typedef std::shared_ptr<BaseAction> BaseActionPtr;
typedef std::shared_ptr<const BaseAction> BaseActionConstPtr;

class BaseAction {
public:
  BaseAction(const radial_menu_model::ActionConstPtr &action) {
    action_.reset(new radial_menu_model::Action(*action));
  }

  virtual void execute() const = 0;

  const int32_t id() const { return action_->id(); }

protected:
  radial_menu_model::ActionConstPtr action_;
};

} // namespace radial_menu_action

#endif 