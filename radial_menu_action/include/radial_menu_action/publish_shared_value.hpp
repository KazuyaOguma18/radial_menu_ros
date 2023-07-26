#ifndef RADIAL_MENU_ACTION_PUBLISH_SHARED_VALUE_HPP
#define RADIAL_MENU_ACTION_PUBLISH_SHARED_VALUE_HPP

#include "radial_menu_action/base_action.hpp"
#include "radial_menu_action/shared.hpp"
#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>
#include <ros/message_traits.h>

namespace radial_menu_action {

class PublishSharedValue : public BaseAction {
public:
  PublishSharedValue(const radial_menu_model::ActionConstPtr &action) : BaseAction(action){}

  virtual bool init() override {
    // Check Elements ------------------------------------------    
    // topic
    if (!get_attribute("topic", topic_)) { return false; }

    // topic_type
    if (!get_attribute("topic_type", topic_type_)) { return false; }

    // key
    if (!get_attribute("key", key_)) { return false; }

    //////////////////////////////////////////////////////////////

    // make Publisher
    if      (topic_type_ == "std_msgs/String")            { makePub< std_msgs::String >(); }
    else if (topic_type_ == "std_msgs/Bool")              { makePub< std_msgs::Bool >(); }
    else if (topic_type_ == "std_msgs/Char")              { makePub< std_msgs::Char >(); }
    else if (topic_type_ == "std_msgs/Float32")           { makePub< std_msgs::Float32 >(); }
    else if (topic_type_ == "std_msgs/Float64")           { makePub< std_msgs::Float64 >(); }
    else if (topic_type_ == "std_msgs/Int8")              { makePub< std_msgs::Int8 >(); }
    else if (topic_type_ == "std_msgs/UInt8")             { makePub< std_msgs::UInt8 >(); }
    else if (topic_type_ == "std_msgs/Int16")             { makePub< std_msgs::Int16 >(); }
    else if (topic_type_ == "std_msgs/UInt16")            { makePub< std_msgs::UInt16 >(); }
    else if (topic_type_ == "std_msgs/Int32")             { makePub< std_msgs::Int32 >(); }
    else if (topic_type_ == "std_msgs/UInt32")            { makePub< std_msgs::UInt32 >(); }
    else if (topic_type_ == "std_msgs/Int64")             { makePub< std_msgs::Int64 >(); }
    else if (topic_type_ == "std_msgs/UInt64")            { makePub< std_msgs::UInt64 >(); }
    else {
      ROS_ERROR_STREAM("radial_menu_action::PublishSharedValue : Invalid topic type ( " + topic_type_ + " )");
      return false;
    }
    return true;

  }

  virtual void execute() const override {
    if      (topic_type_ == "std_msgs/String")            { publish< std_msgs::String >(); }
    else if (topic_type_ == "std_msgs/Bool")              { publish< std_msgs::Bool >(); }
    else if (topic_type_ == "std_msgs/Char")              { publish< std_msgs::Char >(); }
    else if (topic_type_ == "std_msgs/Float32")           { publish< std_msgs::Float32 >(); }
    else if (topic_type_ == "std_msgs/Float64")           { publish< std_msgs::Float64 >(); }
    else if (topic_type_ == "std_msgs/Int8")              { publish< std_msgs::Int8 >(); }
    else if (topic_type_ == "std_msgs/UInt8")             { publish< std_msgs::UInt8 >(); }
    else if (topic_type_ == "std_msgs/Int16")             { publish< std_msgs::Int16 >(); }
    else if (topic_type_ == "std_msgs/UInt16")            { publish< std_msgs::UInt16 >(); }
    else if (topic_type_ == "std_msgs/Int32")             { publish< std_msgs::Int32 >(); }
    else if (topic_type_ == "std_msgs/UInt32")            { publish< std_msgs::UInt32 >(); }
    else if (topic_type_ == "std_msgs/Int64")             { publish< std_msgs::Int64 >(); }
    else if (topic_type_ == "std_msgs/UInt64")            { publish< std_msgs::UInt64 >(); }    
  }

private:
  template < typename M >
  void makePub() {
    pub_ = nh_.advertise< M >(topic_, 10);
  }

  template < typename M >
  void publish() const {
    std::string val_str = Shared::get(key_);
    M msg;
    msg.data = to_value< typename M::_data_type >(val_str);
    pub_.publish(msg);
  }

private:
  std::string topic_, topic_type_, key_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};


} // namespace radial_menu_action

#endif