#ifndef RADIAL_MENU_ACTION_MOVEGROUP_NAMED_STATE_HPP
#define RADIAL_MENU_ACTION_MOVEGROUP_NAMED_STATE_HPP

#include <radial_menu_action/base_action.hpp>
#include <radial_menu_action/shared.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>

namespace mp = moveit::planning_interface;

namespace radial_menu_action {

class MoveGroupNamedState : public BaseAction {
public:
  MoveGroupNamedState(const radial_menu_model::ActionConstPtr &action)
   : BaseAction(action),
     move_group_(){}

  virtual bool init() override {
    // Check Elements ------------------------------------------    
    // state_name
    if (!get_attribute("state_name", state_name_)) { return false; }

    if (!get_attribute("planning_group", planning_group_)) {
      if (!Shared::has("planning_group")) {
        return false;
      }
      planning_group_ = Shared::get_required("planning_group");
    }
    else {
      Shared::add("planning_group", planning_group_);
    }
    //////////////////////////////////////////////////////////////

    move_group_ = std::make_shared< mp::MoveGroupInterface >(planning_group_);
    return true;
  }

  virtual void execute() const override {
    ROS_INFO_STREAM("Moving to '" << state_name_ << "'");
    move_group_->setNamedTarget(state_name_);
    move_group_->asyncMove();
  }

private:
  mp::MoveGroupInterfacePtr move_group_;
  std::string state_name_, planning_group_;
};


} // namespace radial_menu_action

#endif // RADIAL_MENU_ACTION_MOVEGROUP_NAMED_STATE_HPP