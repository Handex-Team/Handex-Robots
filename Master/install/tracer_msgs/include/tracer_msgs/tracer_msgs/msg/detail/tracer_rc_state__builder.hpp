// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tracer_msgs:msg/TracerRCState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_RC_STATE__BUILDER_HPP_
#define TRACER_MSGS__MSG__DETAIL__TRACER_RC_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tracer_msgs/msg/detail/tracer_rc_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tracer_msgs
{

namespace msg
{

namespace builder
{

class Init_TracerRCState_var_a
{
public:
  explicit Init_TracerRCState_var_a(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  ::tracer_msgs::msg::TracerRCState var_a(::tracer_msgs::msg::TracerRCState::_var_a_type arg)
  {
    msg_.var_a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_stick_left_h
{
public:
  explicit Init_TracerRCState_stick_left_h(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  Init_TracerRCState_var_a stick_left_h(::tracer_msgs::msg::TracerRCState::_stick_left_h_type arg)
  {
    msg_.stick_left_h = std::move(arg);
    return Init_TracerRCState_var_a(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_stick_left_v
{
public:
  explicit Init_TracerRCState_stick_left_v(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  Init_TracerRCState_stick_left_h stick_left_v(::tracer_msgs::msg::TracerRCState::_stick_left_v_type arg)
  {
    msg_.stick_left_v = std::move(arg);
    return Init_TracerRCState_stick_left_h(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_stick_right_h
{
public:
  explicit Init_TracerRCState_stick_right_h(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  Init_TracerRCState_stick_left_v stick_right_h(::tracer_msgs::msg::TracerRCState::_stick_right_h_type arg)
  {
    msg_.stick_right_h = std::move(arg);
    return Init_TracerRCState_stick_left_v(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_stick_right_v
{
public:
  explicit Init_TracerRCState_stick_right_v(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  Init_TracerRCState_stick_right_h stick_right_v(::tracer_msgs::msg::TracerRCState::_stick_right_v_type arg)
  {
    msg_.stick_right_v = std::move(arg);
    return Init_TracerRCState_stick_right_h(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_swd
{
public:
  explicit Init_TracerRCState_swd(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  Init_TracerRCState_stick_right_v swd(::tracer_msgs::msg::TracerRCState::_swd_type arg)
  {
    msg_.swd = std::move(arg);
    return Init_TracerRCState_stick_right_v(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_swc
{
public:
  explicit Init_TracerRCState_swc(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  Init_TracerRCState_swd swc(::tracer_msgs::msg::TracerRCState::_swc_type arg)
  {
    msg_.swc = std::move(arg);
    return Init_TracerRCState_swd(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_swb
{
public:
  explicit Init_TracerRCState_swb(::tracer_msgs::msg::TracerRCState & msg)
  : msg_(msg)
  {}
  Init_TracerRCState_swc swb(::tracer_msgs::msg::TracerRCState::_swb_type arg)
  {
    msg_.swb = std::move(arg);
    return Init_TracerRCState_swc(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

class Init_TracerRCState_swa
{
public:
  Init_TracerRCState_swa()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TracerRCState_swb swa(::tracer_msgs::msg::TracerRCState::_swa_type arg)
  {
    msg_.swa = std::move(arg);
    return Init_TracerRCState_swb(msg_);
  }

private:
  ::tracer_msgs::msg::TracerRCState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tracer_msgs::msg::TracerRCState>()
{
  return tracer_msgs::msg::builder::Init_TracerRCState_swa();
}

}  // namespace tracer_msgs

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_RC_STATE__BUILDER_HPP_
