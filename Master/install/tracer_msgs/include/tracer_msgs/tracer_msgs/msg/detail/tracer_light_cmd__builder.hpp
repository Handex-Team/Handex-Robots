// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tracer_msgs:msg/TracerLightCmd.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_CMD__BUILDER_HPP_
#define TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tracer_msgs/msg/detail/tracer_light_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tracer_msgs
{

namespace msg
{

namespace builder
{

class Init_TracerLightCmd_rear_custom_value
{
public:
  explicit Init_TracerLightCmd_rear_custom_value(::tracer_msgs::msg::TracerLightCmd & msg)
  : msg_(msg)
  {}
  ::tracer_msgs::msg::TracerLightCmd rear_custom_value(::tracer_msgs::msg::TracerLightCmd::_rear_custom_value_type arg)
  {
    msg_.rear_custom_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tracer_msgs::msg::TracerLightCmd msg_;
};

class Init_TracerLightCmd_rear_mode
{
public:
  explicit Init_TracerLightCmd_rear_mode(::tracer_msgs::msg::TracerLightCmd & msg)
  : msg_(msg)
  {}
  Init_TracerLightCmd_rear_custom_value rear_mode(::tracer_msgs::msg::TracerLightCmd::_rear_mode_type arg)
  {
    msg_.rear_mode = std::move(arg);
    return Init_TracerLightCmd_rear_custom_value(msg_);
  }

private:
  ::tracer_msgs::msg::TracerLightCmd msg_;
};

class Init_TracerLightCmd_front_custom_value
{
public:
  explicit Init_TracerLightCmd_front_custom_value(::tracer_msgs::msg::TracerLightCmd & msg)
  : msg_(msg)
  {}
  Init_TracerLightCmd_rear_mode front_custom_value(::tracer_msgs::msg::TracerLightCmd::_front_custom_value_type arg)
  {
    msg_.front_custom_value = std::move(arg);
    return Init_TracerLightCmd_rear_mode(msg_);
  }

private:
  ::tracer_msgs::msg::TracerLightCmd msg_;
};

class Init_TracerLightCmd_front_mode
{
public:
  explicit Init_TracerLightCmd_front_mode(::tracer_msgs::msg::TracerLightCmd & msg)
  : msg_(msg)
  {}
  Init_TracerLightCmd_front_custom_value front_mode(::tracer_msgs::msg::TracerLightCmd::_front_mode_type arg)
  {
    msg_.front_mode = std::move(arg);
    return Init_TracerLightCmd_front_custom_value(msg_);
  }

private:
  ::tracer_msgs::msg::TracerLightCmd msg_;
};

class Init_TracerLightCmd_cmd_ctrl_allowed
{
public:
  Init_TracerLightCmd_cmd_ctrl_allowed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TracerLightCmd_front_mode cmd_ctrl_allowed(::tracer_msgs::msg::TracerLightCmd::_cmd_ctrl_allowed_type arg)
  {
    msg_.cmd_ctrl_allowed = std::move(arg);
    return Init_TracerLightCmd_front_mode(msg_);
  }

private:
  ::tracer_msgs::msg::TracerLightCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tracer_msgs::msg::TracerLightCmd>()
{
  return tracer_msgs::msg::builder::Init_TracerLightCmd_cmd_ctrl_allowed();
}

}  // namespace tracer_msgs

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_CMD__BUILDER_HPP_
