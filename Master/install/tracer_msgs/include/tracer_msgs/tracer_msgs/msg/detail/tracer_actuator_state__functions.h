// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tracer_msgs:msg/TracerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__FUNCTIONS_H_
#define TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tracer_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "tracer_msgs/msg/detail/tracer_actuator_state__struct.h"

/// Initialize msg/TracerActuatorState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tracer_msgs__msg__TracerActuatorState
 * )) before or use
 * tracer_msgs__msg__TracerActuatorState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
bool
tracer_msgs__msg__TracerActuatorState__init(tracer_msgs__msg__TracerActuatorState * msg);

/// Finalize msg/TracerActuatorState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
void
tracer_msgs__msg__TracerActuatorState__fini(tracer_msgs__msg__TracerActuatorState * msg);

/// Create msg/TracerActuatorState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tracer_msgs__msg__TracerActuatorState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
tracer_msgs__msg__TracerActuatorState *
tracer_msgs__msg__TracerActuatorState__create();

/// Destroy msg/TracerActuatorState message.
/**
 * It calls
 * tracer_msgs__msg__TracerActuatorState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
void
tracer_msgs__msg__TracerActuatorState__destroy(tracer_msgs__msg__TracerActuatorState * msg);

/// Check for msg/TracerActuatorState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
bool
tracer_msgs__msg__TracerActuatorState__are_equal(const tracer_msgs__msg__TracerActuatorState * lhs, const tracer_msgs__msg__TracerActuatorState * rhs);

/// Copy a msg/TracerActuatorState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
bool
tracer_msgs__msg__TracerActuatorState__copy(
  const tracer_msgs__msg__TracerActuatorState * input,
  tracer_msgs__msg__TracerActuatorState * output);

/// Initialize array of msg/TracerActuatorState messages.
/**
 * It allocates the memory for the number of elements and calls
 * tracer_msgs__msg__TracerActuatorState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
bool
tracer_msgs__msg__TracerActuatorState__Sequence__init(tracer_msgs__msg__TracerActuatorState__Sequence * array, size_t size);

/// Finalize array of msg/TracerActuatorState messages.
/**
 * It calls
 * tracer_msgs__msg__TracerActuatorState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
void
tracer_msgs__msg__TracerActuatorState__Sequence__fini(tracer_msgs__msg__TracerActuatorState__Sequence * array);

/// Create array of msg/TracerActuatorState messages.
/**
 * It allocates the memory for the array and calls
 * tracer_msgs__msg__TracerActuatorState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
tracer_msgs__msg__TracerActuatorState__Sequence *
tracer_msgs__msg__TracerActuatorState__Sequence__create(size_t size);

/// Destroy array of msg/TracerActuatorState messages.
/**
 * It calls
 * tracer_msgs__msg__TracerActuatorState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
void
tracer_msgs__msg__TracerActuatorState__Sequence__destroy(tracer_msgs__msg__TracerActuatorState__Sequence * array);

/// Check for msg/TracerActuatorState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
bool
tracer_msgs__msg__TracerActuatorState__Sequence__are_equal(const tracer_msgs__msg__TracerActuatorState__Sequence * lhs, const tracer_msgs__msg__TracerActuatorState__Sequence * rhs);

/// Copy an array of msg/TracerActuatorState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tracer_msgs
bool
tracer_msgs__msg__TracerActuatorState__Sequence__copy(
  const tracer_msgs__msg__TracerActuatorState__Sequence * input,
  tracer_msgs__msg__TracerActuatorState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__FUNCTIONS_H_
