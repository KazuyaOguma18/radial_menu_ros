#ifndef RADIAL_MENU_ACTION_PUBLISH_HPP
#define RADIAL_MENU_ACTION_PUBLISH_HPP

#include <radial_menu_action/base_action.hpp>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

namespace radial_menu_action {
class Publish : public BaseAction {
public:
  Publish(const radial_menu_model::ActionConstPtr &action) : BaseAction(action){
    if (action_->topic_type() == "std_msgs/String") {
      pub_ = nh_.advertise<std_msgs::String>(action_->topic(), 10);
    }
    else if (action_->topic_type() == "std_msgs/Float32") {
      pub_ = nh_.advertise<std_msgs::Float32>(action_->topic(), 10);
    }
    else if (action_->topic_type() == "std_msgs/Float64") {
      pub_ = nh_.advertise<std_msgs::Float64>(action_->topic(), 10);
    }
    else {
      throw ros::Exception("radial_menu_action::Publish : Invalid topic type ( " + action_->topic_type() + " )");
    }
  }

  virtual void execute() const override {
    // ROS_INFO("Publish::execute()");
    if (action_->topic_type() == "std_msgs/String") {
      std_msgs::String msg;
      msg.data = action_->values();
      pub_.publish(msg);
    }

    else if (action_->topic_type() == "std_msgs/Float32") {
      std_msgs::Float32 msg;
      msg.data = action_->values<float>()[0];
      pub_.publish(msg);
    }

    else if (action_->topic_type() == "std_msgs/Float64") {
      std_msgs::Float64 msg;
      msg.data = action_->values<double>()[0];
      pub_.publish(msg);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};


} // namespace radial_menu_action

#endif 