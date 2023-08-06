#ifndef RADIAL_MENU_ACTION_FLIPPER_NAMED_STATE
#define RADIAL_MENU_ACTION_FLIPPER_NAMED_STATE

#include <radial_menu_action/base_action.hpp>
#include <radial_menu_action/shared.hpp>
#include <std_msgs/Float64.h>

#include <ros/param.h>
#include <ros/names.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

namespace radial_menu_action {

class FlipperNamedState : public BaseAction {
public:
  FlipperNamedState(const radial_menu_model::ActionConstPtr &action) : BaseAction(action){}

  virtual bool init() override {
    // Check Elements ------------------------------------------ 
    // fr_topic
    if (!getTopic("fr_topic", fr_topic_)) { return false; }

    // fl_topic
    if (!getTopic("fl_topic", fl_topic_)) { return false; }

    // rr_topic
    if (!getTopic("rr_topic", rr_topic_)) { return false; }

    // rl_topic
    if (!getTopic("rl_topic", rl_topic_)) { return false; }

    // fr position
    if (!get_attribute("fr", fr_str_)) { return false; }

    // fl position
    if (!get_attribute("fl", fl_str_)) { return false; }

    // rr position
    if (!get_attribute("rr", rr_str_)) { return false; }

    // rl position
    if (!get_attribute("rl", rl_str_)) { return false; }

    // state_name
    if (!get_attribute("state_name", state_name_)) { return false; }

    //////////////////////////////////////////////////////////////

    // Publisher
    ros::NodeHandle nh;
    fr_pub_ = nh.advertise< std_msgs::Float64 >(fr_topic_, 1, true);
    fl_pub_ = nh.advertise< std_msgs::Float64 >(fl_topic_, 1, true);
    rr_pub_ = nh.advertise< std_msgs::Float64 >(rr_topic_, 1, true);
    rl_pub_ = nh.advertise< std_msgs::Float64 >(rl_topic_, 1, true);

    return true;
  }

  virtual void execute() const override {
    ROS_INFO_STREAM("FlipperNamedState : Moving to '" << state_name_ << "'");
    fr_pub_.publish(makeFloat64(to_value<double>(fr_str_)));
    fl_pub_.publish(makeFloat64(to_value<double>(fl_str_)));
    rr_pub_.publish(makeFloat64(to_value<double>(rr_str_)));
    rl_pub_.publish(makeFloat64(to_value<double>(rl_str_)));
  }

private:
  inline const bool getTopic(const std::string& key, std::string& topic) {
    if (!get_attribute(key, topic)) {
      if (!Shared::has(key)) {
        return false;
      }
      topic = Shared::get_required(key);
    }
    else {
      Shared::add(key, topic);
    }
    return true;    
  }

  inline const std_msgs::Float64 makeFloat64(const double& data) const {
    std_msgs::Float64 msg;
    msg.data = data;
    return msg;
  }


private:
  std::string fr_topic_, fl_topic_, rr_topic_, rl_topic_;
  std::string fr_str_, fl_str_, rr_str_, rl_str_;
  ros::Publisher fr_pub_, fl_pub_, rr_pub_, rl_pub_;
  std::string state_name_;
};

} // namespace radial_menu_action

#endif // RADIAL_MENU_ACTION_FLIPPER_NAMED_STATE