#ifndef RADIAL_MENU_ACTION_SERVICE_HPP
#define RADIAL_MENU_ACTION_SERVICE_HPP

#include <radial_menu_action/base_action.hpp>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <ros/ros.h>

namespace radial_menu_action {

class Service : public BaseAction {
public:
  Service(const radial_menu_model::ActionConstPtr &action) : BaseAction(action){}

  virtual bool init() override {
    // Check Elements ------------------------------------------    
    // topic
    if (!get_attribute("topic", topic_)) { return false; }

    // topic_type
    if (!get_attribute("topic_type", topic_type_)) { return false; }

    // key
    if (!get_attribute("values", values_)) { return false; }

    //////////////////////////////////////////////////////////////

    
    if (topic_type_ == "std_srvs/Empty")        { makeClient<std_srvs::Empty>(); }
    else if (topic_type_ == "std_srvs/SetBool") { makeClient<std_srvs::SetBool>(); }
    else if (topic_type_ == "std_srvs/Trigger") { makeClient<std_srvs::Trigger>(); }
    else { return false; }
    return true;
  } 

  virtual void execute() const override;

private:
  template < typename M >
  void makeClient() {
    client_ = nh_.serviceClient< M >(topic_);
  }

  template < typename M >
  bool call() const {
    M srv;
    if (client_.call(srv)) {
      return true;
    }
    ROS_ERROR_STREAM("Failed to call service : '" << topic_ << "'");
    return false;
  }

private:
  ros::NodeHandle nh_;
  mutable ros::ServiceClient client_;
  std::string topic_, topic_type_, values_;
};

template <>
bool Service::call< std_srvs::SetBool >() const {
  std_srvs::SetBool srv;
  srv.request.data = to_value< bool >(values_);
  if (client_.call(srv)) {
    ROS_INFO_STREAM("Recieved Data: " << "\nsuccess : " << srv.response.success 
                                      << "\nmessage : " << srv.response.message);
    return true;
  }
  ROS_ERROR_STREAM("Failed to call service : '" << topic_ << "'");
  return false;
}

template <>
bool Service::call< std_srvs::Trigger >() const {
  std_srvs::Trigger srv;
  if (client_.call(srv)) {
    ROS_INFO_STREAM("Recieved Data: " << "\nsuccess : " << srv.response.success 
                                      << "\nmessage : " << srv.response.message);
    return true;
  }
  ROS_ERROR_STREAM("Failed to call service : '" << topic_ << "'");
  return false;
}

void Service::execute() const {
  if (!ros::service::waitForService(topic_, ros::Duration(3.0))){
    ROS_ERROR_STREAM("Service is not available : '" << topic_ << "'");
    return;
  }

  if (topic_type_ == "std_srvs/Empty")        { call<std_srvs::Empty>(); }
  else if (topic_type_ == "std_srvs/SetBool") { call<std_srvs::SetBool>(); }
  else if (topic_type_ == "std_srvs/Trigger") { call<std_srvs::Trigger>(); }
}

} // namespace radial_menu_action

#endif