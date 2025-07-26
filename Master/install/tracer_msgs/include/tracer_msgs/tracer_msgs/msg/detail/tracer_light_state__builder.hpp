// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tracer_msgs:msg/TracerLightState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__BUILDER_HPP_
#define TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tracer_msgs/msg/detail/tracer_light_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tracer_msgs
{

namespace msg
{

namespace builder
{

class Init_TracerLightState_custom_value
{
public:
  explicit Init_TracerLightState_custom_value(::tracer_msgs::msg::TracerLightState & msg)
  : msg_(msg)
  {}
  ::tracer_msgs::msg::TracerLightState custom_value(::tracer_msgs::msg::TracerLightState::_custom_value_type arg)
  {
    msg_.custom_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tracer_msgs::msg::TracerLightState msg_;
};

class Init_TracerLightState_mode
{
public:
  Init_TracerLightState_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TracerLightState_custom_value mode(::tracer_msgs::msg::TracerLightState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_TracerLightState_custom_value(msg_);
  }

private:
  ::tracer_msgs::msg::TracerLightState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tracer_msgs::msg::TracerLightState>()
{
  return tracer_msgs::msg::builder::Init_TracerLightState_mode();
}

}  // namespace tracer_msgs

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__BUILDER_HPP_
