// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tracer_msgs:msg/TracerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__TRAITS_HPP_
#define TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tracer_msgs/msg/detail/tracer_actuator_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tracer_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TracerActuatorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: rpm
  {
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TracerActuatorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TracerActuatorState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tracer_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tracer_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tracer_msgs::msg::TracerActuatorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  tracer_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tracer_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tracer_msgs::msg::TracerActuatorState & msg)
{
  return tracer_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tracer_msgs::msg::TracerActuatorState>()
{
  return "tracer_msgs::msg::TracerActuatorState";
}

template<>
inline const char * name<tracer_msgs::msg::TracerActuatorState>()
{
  return "tracer_msgs/msg/TracerActuatorState";
}

template<>
struct has_fixed_size<tracer_msgs::msg::TracerActuatorState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tracer_msgs::msg::TracerActuatorState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tracer_msgs::msg::TracerActuatorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__TRAITS_HPP_
