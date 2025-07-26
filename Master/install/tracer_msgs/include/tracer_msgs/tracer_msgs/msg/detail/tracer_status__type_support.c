// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tracer_msgs:msg/TracerStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tracer_msgs/msg/detail/tracer_status__rosidl_typesupport_introspection_c.h"
#include "tracer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tracer_msgs/msg/detail/tracer_status__functions.h"
#include "tracer_msgs/msg/detail/tracer_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `actuator_states`
#include "tracer_msgs/msg/tracer_actuator_state.h"
// Member `actuator_states`
#include "tracer_msgs/msg/detail/tracer_actuator_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tracer_msgs__msg__TracerStatus__init(message_memory);
}

void tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_fini_function(void * message_memory)
{
  tracer_msgs__msg__TracerStatus__fini(message_memory);
}

size_t tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__size_function__TracerStatus__actuator_states(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__get_const_function__TracerStatus__actuator_states(
  const void * untyped_member, size_t index)
{
  const tracer_msgs__msg__TracerActuatorState * member =
    (const tracer_msgs__msg__TracerActuatorState *)(untyped_member);
  return &member[index];
}

void * tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__get_function__TracerStatus__actuator_states(
  void * untyped_member, size_t index)
{
  tracer_msgs__msg__TracerActuatorState * member =
    (tracer_msgs__msg__TracerActuatorState *)(untyped_member);
  return &member[index];
}

void tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__fetch_function__TracerStatus__actuator_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const tracer_msgs__msg__TracerActuatorState * item =
    ((const tracer_msgs__msg__TracerActuatorState *)
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__get_const_function__TracerStatus__actuator_states(untyped_member, index));
  tracer_msgs__msg__TracerActuatorState * value =
    (tracer_msgs__msg__TracerActuatorState *)(untyped_value);
  *value = *item;
}

void tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__assign_function__TracerStatus__actuator_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  tracer_msgs__msg__TracerActuatorState * item =
    ((tracer_msgs__msg__TracerActuatorState *)
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__get_function__TracerStatus__actuator_states(untyped_member, index));
  const tracer_msgs__msg__TracerActuatorState * value =
    (const tracer_msgs__msg__TracerActuatorState *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, control_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, battery_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actuator_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, actuator_states),  // bytes offset in struct
    NULL,  // default value
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__size_function__TracerStatus__actuator_states,  // size() function pointer
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__get_const_function__TracerStatus__actuator_states,  // get_const(index) function pointer
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__get_function__TracerStatus__actuator_states,  // get(index) function pointer
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__fetch_function__TracerStatus__actuator_states,  // fetch(index, &value) function pointer
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__assign_function__TracerStatus__actuator_states,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "light_control_enabled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tracer_msgs__msg__TracerStatus, light_control_enabled),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_members = {
  "tracer_msgs__msg",  // message namespace
  "TracerStatus",  // message name
  8,  // number of fields
  sizeof(tracer_msgs__msg__TracerStatus),
  tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_member_array,  // message members
  tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_type_support_handle = {
  0,
  &tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tracer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tracer_msgs, msg, TracerStatus)() {
  tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tracer_msgs, msg, TracerActuatorState)();
  if (!tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_type_support_handle.typesupport_identifier) {
    tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tracer_msgs__msg__TracerStatus__rosidl_typesupport_introspection_c__TracerStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
