// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tracer_msgs:msg/TracerLightCmd.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_CMD__STRUCT_H_
#define TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LIGHT_CONST_OFF'.
enum
{
  tracer_msgs__msg__TracerLightCmd__LIGHT_CONST_OFF = 0
};

/// Constant 'LIGHT_CONST_ON'.
enum
{
  tracer_msgs__msg__TracerLightCmd__LIGHT_CONST_ON = 1
};

/// Constant 'LIGHT_BREATH'.
enum
{
  tracer_msgs__msg__TracerLightCmd__LIGHT_BREATH = 2
};

/// Constant 'LIGHT_CUSTOM'.
enum
{
  tracer_msgs__msg__TracerLightCmd__LIGHT_CUSTOM = 3
};

/// Struct defined in msg/TracerLightCmd in the package tracer_msgs.
typedef struct tracer_msgs__msg__TracerLightCmd
{
  bool cmd_ctrl_allowed;
  uint8_t front_mode;
  uint8_t front_custom_value;
  uint8_t rear_mode;
  uint8_t rear_custom_value;
} tracer_msgs__msg__TracerLightCmd;

// Struct for a sequence of tracer_msgs__msg__TracerLightCmd.
typedef struct tracer_msgs__msg__TracerLightCmd__Sequence
{
  tracer_msgs__msg__TracerLightCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tracer_msgs__msg__TracerLightCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_CMD__STRUCT_H_
