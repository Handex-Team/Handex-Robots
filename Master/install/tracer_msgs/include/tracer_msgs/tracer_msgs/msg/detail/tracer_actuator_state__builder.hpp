// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tracer_msgs:msg/TracerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__BUILDER_HPP_
#define TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tracer_msgs/msg/detail/tracer_actuator_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tracer_msgs
{

namespace msg
{

namespace builder
{

class Init_TracerActuatorState_current
{
public:
  explicit Init_TracerActuatorState_current(::tracer_msgs::msg::TracerActuatorState & msg)
  : msg_(msg)
  {}
  ::tracer_msgs::msg::TracerActuatorState current(::tracer_msgs::msg::TracerActuatorState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tracer_msgs::msg::TracerActuatorState msg_;
};

class Init_TracerActuatorState_rpm
{
public:
  Init_TracerActuatorState_rpm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TracerActuatorState_current rpm(::tracer_msgs::msg::TracerActuatorState::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_TracerActuatorState_current(msg_);
  }

private:
  ::tracer_msgs::msg::TracerActuatorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tracer_msgs::msg::TracerActuatorState>()
{
  return tracer_msgs::msg::builder::Init_TracerActuatorState_rpm();
}

}  // namespace tracer_msgs

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__BUILDER_HPP_
