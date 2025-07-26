// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tracer_msgs:msg/TracerLightState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__STRUCT_H_
#define TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TracerLightState in the package tracer_msgs.
typedef struct tracer_msgs__msg__TracerLightState
{
  uint8_t mode;
  uint8_t custom_value;
} tracer_msgs__msg__TracerLightState;

// Struct for a sequence of tracer_msgs__msg__TracerLightState.
typedef struct tracer_msgs__msg__TracerLightState__Sequence
{
  tracer_msgs__msg__TracerLightState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tracer_msgs__msg__TracerLightState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__STRUCT_H_
