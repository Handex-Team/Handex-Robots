// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tracer_msgs:msg/TracerStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tracer_msgs/msg/detail/tracer_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tracer_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TracerStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tracer_msgs::msg::TracerStatus(_init);
}

void TracerStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tracer_msgs::msg::TracerStatus *>(message_memory);
  typed_message->~TracerStatus();
}

size_t size_function__TracerStatus__actuator_states(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__TracerStatus__actuator_states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<tracer_msgs::msg::TracerActuatorState, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__TracerStatus__actuator_states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<tracer_msgs::msg::TracerActuatorState, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__TracerStatus__actuator_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const tracer_msgs::msg::TracerActuatorState *>(
    get_const_function__TracerStatus__actuator_states(untyped_member, index));
  auto & value = *reinterpret_cast<tracer_msgs::msg::TracerActuatorState *>(untyped_value);
  value = item;
}

void assign_function__TracerStatus__actuator_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<tracer_msgs::msg::TracerActuatorState *>(
    get_function__TracerStatus__actuator_states(untyped_member, index));
  const auto & value = *reinterpret_cast<const tracer_msgs::msg::TracerActuatorState *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TracerStatus_message_member_array[8] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, linear_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, angular_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "control_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, control_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error_code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, error_code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, battery_voltage),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "actuator_states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<tracer_msgs::msg::TracerActuatorState>(),  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, actuator_states),  // bytes offset in struct
    nullptr,  // default value
    size_function__TracerStatus__actuator_states,  // size() function pointer
    get_const_function__TracerStatus__actuator_states,  // get_const(index) function pointer
    get_function__TracerStatus__actuator_states,  // get(index) function pointer
    fetch_function__TracerStatus__actuator_states,  // fetch(index, &value) function pointer
    assign_function__TracerStatus__actuator_states,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "light_control_enabled",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs::msg::TracerStatus, light_control_enabled),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TracerStatus_message_members = {
  "tracer_msgs::msg",  // message namespace
  "TracerStatus",  // message name
  8,  // number of fields
  sizeof(tracer_msgs::msg::TracerStatus),
  TracerStatus_message_member_array,  // message members
  TracerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  TracerStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TracerStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TracerStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tracer_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tracer_msgs::msg::TracerStatus>()
{
  return &::tracer_msgs::msg::rosidl_typesupport_introspection_cpp::TracerStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tracer_msgs, msg, TracerStatus)() {
  return &::tracer_msgs::msg::rosidl_typesupport_introspection_cpp::TracerStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
