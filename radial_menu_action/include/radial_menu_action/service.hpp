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
  Service(const radial_menu_model::ActionConstPtr &action) : BaseAction(action){
    if (action_->topic_type() == "std_srvs/Empty")        { makeClient<std_srvs::Empty>(); }
    else if (action_->topic_type() == "std_srvs/SetBool") { makeClient<std_srvs::SetBool>(); }
    else if (action_->topic_type() == "std_srvs/Trigger") { makeClient<std_srvs::Trigger>(); }
  }

  virtual void execute() const override;

private:
  template < typename M >
  void makeClient() {
    client_ = nh_.serviceClient< M >(action_->topic());
  }

  template < typename M >
  bool call() const {
    M srv;
    if (client_.call(srv)) {
      return true;
    }
    ROS_ERROR_STREAM("Failed to call service : '" << action_->topic() << "'");
    return false;
  }

private:
  ros::NodeHandle nh_;
  mutable ros::ServiceClient client_;
};

template <>
bool Service::call< std_srvs::SetBool >() const {
  std_srvs::SetBool srv;
  srv.request.data = action_->values< bool >()[0];
  if (client_.call(srv)) {
    ROS_INFO_STREAM("Recieved Data: " << "\nsuccess : " << srv.response.success 
                                      << "\nmessage : " << srv.response.message);
    return true;
  }
  ROS_ERROR_STREAM("Failed to call service : '" << action_->topic() << "'");
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
  ROS_ERROR_STREAM("Failed to call service : '" << action_->topic() << "'");
  return false;
}

void Service::execute() const {
  if (!ros::service::waitForService(action_->topic(), ros::Duration(3.0))){
    ROS_ERROR_STREAM("Service is not available : '" << action_->topic() << "'");
    return;
  }

  if (action_->topic_type() == "std_srvs/Empty")        { call<std_srvs::Empty>(); }
  else if (action_->topic_type() == "std_srvs/SetBool") { call<std_srvs::SetBool>(); }
  else if (action_->topic_type() == "std_srvs/Trigger") { call<std_srvs::Trigger>(); }
}

} // namespace radial_menu_action

#endif