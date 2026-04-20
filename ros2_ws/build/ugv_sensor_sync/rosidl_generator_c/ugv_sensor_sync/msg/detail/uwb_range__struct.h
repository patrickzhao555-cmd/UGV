// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ugv_sensor_sync:msg/UwbRange.idl
// generated code does not contain a copyright notice

#ifndef UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__STRUCT_H_
#define UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/UwbRange in the package ugv_sensor_sync.
typedef struct ugv_sensor_sync__msg__UwbRange
{
  std_msgs__msg__Header header;
  uint64_t esp32_time_us;
  double range_m;
  double range_stddev_m;
  uint32_t anchor_id;
} ugv_sensor_sync__msg__UwbRange;

// Struct for a sequence of ugv_sensor_sync__msg__UwbRange.
typedef struct ugv_sensor_sync__msg__UwbRange__Sequence
{
  ugv_sensor_sync__msg__UwbRange * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_sensor_sync__msg__UwbRange__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__STRUCT_H_
