#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <radial_menu_action/publish.hpp>
#include <radial_menu_action/service.hpp>

#include <radial_menu_msgs/State.h>

#include <radial_menu_model/model.hpp>

namespace radial_menu_action {
class RadialMenuAction : public nodelet::Nodelet {
public:
  virtual void onInit() {
    NODELET_INFO("RadialMenuAction::onInit()");
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    model_.reset(new radial_menu_model::Model());

    if (!model_->setDescriptionFromParam(nh_.resolveName("menu_description"))) {
      throw ros::Exception("Cannot set a model description from the param '" +
                           nh_.resolveName("menu_description") + "'");
    }

    for (const auto& action : model_->getActions()) {
      ROS_INFO_STREAM("RadialMenuAction::onInit(), action_id : " << action->id());
      if (action->type() == "publish") {
        actions_.push_back(std::make_shared<Publish>(action));
      }
      else if (action->type() == "service") {
        actions_.push_back(std::make_shared<Service>(action));
      }
    }

    for (auto& action : actions_) {
      if(!action->init()) {
        throw ros::Exception("Failed to action initialize");
      }
    }

    state_sub_ = nh_.subscribe<radial_menu_msgs::State>("menu_state", 1, &RadialMenuAction::stateCallback, this);
  }

  void stateCallback(const radial_menu_msgs::StateConstPtr &msg) {

    static int last_pointed_id = -1;

    // check last enabled state
    static bool last_enabled = false;
    if (!(!msg->is_enabled && last_enabled)){
      last_enabled = msg->is_enabled;
      last_pointed_id = msg->pointed_id;
      return;
    }
    last_enabled = msg->is_enabled;

    // Search menu from id
    for (const auto &action : actions_) {
      if (action->id() == last_pointed_id) {
        action->execute();
      }
    }
  }

protected:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber state_sub_;
  radial_menu_model::ModelPtr model_;

   std::vector< BaseActionPtr > actions_;
};
} // namespace radial_menu_action

PLUGINLIB_EXPORT_CLASS(radial_menu_action::RadialMenuAction, nodelet::Nodelet);