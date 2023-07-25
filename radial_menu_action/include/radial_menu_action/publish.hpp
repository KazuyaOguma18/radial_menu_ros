#ifndef RADIAL_MENU_ACTION_PUBLISH_HPP
#define RADIAL_MENU_ACTION_PUBLISH_HPP

#include <radial_menu_action/base_action.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/String.h>
#include <ros/message_traits.h>

#include <type_traits>

namespace radial_menu_action {
template< class >
struct is_vector : std::false_type {};

template < class T, class ALLOCATOR >
struct is_vector< std::vector< T, ALLOCATOR > > : std::true_type {};

template <typename T>
struct is_data_type {
private:
    template <typename C>
    static constexpr std::true_type test(typename C::_data_type*);

    template <typename>
    static constexpr std::false_type test(...);

public:
    static constexpr bool value = decltype(test<T>(nullptr))::value;
};

class Publish : public BaseAction {
public:
  Publish(const radial_menu_model::ActionConstPtr &action) : BaseAction(action){

  }

  virtual bool init() override {
    // Check Elements 


    //


    if      (action_->topic_type() == "std_msgs/String")            { makePub< std_msgs::String >(); }
    else if (action_->topic_type() == "std_msgs/Bool")              { makePub< std_msgs::Bool >(); }
    else if (action_->topic_type() == "std_msgs/Byte")              { makePub< std_msgs::Byte >(); }
    else if (action_->topic_type() == "std_msgs/ByteMultiArray")    { makePub< std_msgs::ByteMultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Char")              { makePub< std_msgs::Char >(); }
    else if (action_->topic_type() == "std_msgs/Empty")             { makePub< std_msgs::Empty >(); }
    else if (action_->topic_type() == "std_msgs/Float32")           { makePub< std_msgs::Float32 >(); }
    else if (action_->topic_type() == "std_msgs/Float32MultiArray") { makePub< std_msgs::Float32MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Float64")           { makePub< std_msgs::Float64 >(); }
    else if (action_->topic_type() == "std_msgs/Float64MultiArray") { makePub< std_msgs::Float64MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int8")              { makePub< std_msgs::Int8 >(); }
    else if (action_->topic_type() == "std_msgs/Int8MultiArray")    { makePub< std_msgs::Int8MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt8")             { makePub< std_msgs::UInt8 >(); }
    else if (action_->topic_type() == "std_msgs/UInt8MultiArray")   { makePub< std_msgs::UInt8MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int16")             { makePub< std_msgs::Int16 >(); }
    else if (action_->topic_type() == "std_msgs/Int16MultiArray")   { makePub< std_msgs::Int16MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt16")            { makePub< std_msgs::UInt16 >(); }
    else if (action_->topic_type() == "std_msgs/UInt16MultiArray")  { makePub< std_msgs::UInt16MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int32")             { makePub< std_msgs::Int32 >(); }
    else if (action_->topic_type() == "std_msgs/Int32MultiArray")   { makePub< std_msgs::Int32MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt32")            { makePub< std_msgs::UInt32 >(); }
    else if (action_->topic_type() == "std_msgs/UInt32MultiArray")  { makePub< std_msgs::UInt32MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int64")             { makePub< std_msgs::Int64 >(); }
    else if (action_->topic_type() == "std_msgs/Int64MultiArray")   { makePub< std_msgs::Int64MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt64")            { makePub< std_msgs::UInt64 >(); }
    else if (action_->topic_type() == "std_msgs/UInt64MultiArray")  { makePub< std_msgs::UInt64MultiArray >(); }
    else {
      ROS_ERROR_STREAM("radial_menu_action::Publish : Invalid topic type ( " + action_->topic_type() + " )");
      return false;
    }
    return true;
  }

  virtual void execute() const override {
    if      (action_->topic_type() == "std_msgs/String")            { publish< std_msgs::String >(); }
    else if (action_->topic_type() == "std_msgs/Bool")              { publish< std_msgs::Bool >(); }
    else if (action_->topic_type() == "std_msgs/Byte")              { publish< std_msgs::Byte >(); }
    else if (action_->topic_type() == "std_msgs/ByteMultiArray")    { publish< std_msgs::ByteMultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Char")              { publish< std_msgs::Char >(); }
    else if (action_->topic_type() == "std_msgs/Empty")             { publish< std_msgs::Empty >(); }
    else if (action_->topic_type() == "std_msgs/Float32")           { publish< std_msgs::Float32 >(); }
    else if (action_->topic_type() == "std_msgs/Float32MultiArray") { publish< std_msgs::Float32MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Float64")           { publish< std_msgs::Float64 >(); }
    else if (action_->topic_type() == "std_msgs/Float64MultiArray") { publish< std_msgs::Float64MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int8")              { publish< std_msgs::Int8 >(); }
    else if (action_->topic_type() == "std_msgs/Int8MultiArray")    { publish< std_msgs::Int8MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt8")             { publish< std_msgs::UInt8 >(); }
    else if (action_->topic_type() == "std_msgs/UInt8MultiArray")   { publish< std_msgs::UInt8MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int16")             { publish< std_msgs::Int16 >(); }
    else if (action_->topic_type() == "std_msgs/Int16MultiArray")   { publish< std_msgs::Int16MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt16")            { publish< std_msgs::UInt16 >(); }
    else if (action_->topic_type() == "std_msgs/UInt16MultiArray")  { publish< std_msgs::UInt16MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int32")             { publish< std_msgs::Int32 >(); }
    else if (action_->topic_type() == "std_msgs/Int32MultiArray")   { publish< std_msgs::Int32MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt32")            { publish< std_msgs::UInt32 >(); }
    else if (action_->topic_type() == "std_msgs/UInt32MultiArray")  { publish< std_msgs::UInt32MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/Int64")             { publish< std_msgs::Int64 >(); }
    else if (action_->topic_type() == "std_msgs/Int64MultiArray")   { publish< std_msgs::Int64MultiArray >(); }
    else if (action_->topic_type() == "std_msgs/UInt64")            { publish< std_msgs::UInt64 >(); }
    else if (action_->topic_type() == "std_msgs/UInt64MultiArray")  { publish< std_msgs::UInt64MultiArray >(); }
  }

private:
  template < typename M >
  void makePub() {
    pub_ = nh_.advertise< M >(action_->topic(), 10);
  }

  // array
  template < typename M >
  typename std::enable_if<is_vector< typename M::_data_type >::value>::type
  publish() const {
    M msg;
    msg.data = action_->values< typename M::_data_type::value_type >();
    // msg.data = action_->values< double >();
    pub_.publish(msg);
  }

  // non array
  template < typename M >
  typename std::enable_if<!is_vector< typename M::_data_type >::value && is_data_type< M >::value>::type
  publish() const {
    M msg;
    msg.data = action_->values< typename M::_data_type >()[0];
    pub_.publish(msg);
  }

  // std_msgs::Empty
  template < typename M >
  typename std::enable_if<!is_data_type< M >::value>::type
  publish() const {
    M msg;
    pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};


} // namespace radial_menu_action

#endif 