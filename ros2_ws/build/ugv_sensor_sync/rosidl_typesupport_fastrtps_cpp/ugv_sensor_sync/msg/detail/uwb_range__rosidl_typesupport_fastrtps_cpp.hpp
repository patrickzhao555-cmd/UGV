// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ugv_sensor_sync:msg/UwbRange.idl
// generated code does not contain a copyright notice

#ifndef UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ugv_sensor_sync/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ugv_sensor_sync/msg/detail/uwb_range__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ugv_sensor_sync
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ugv_sensor_sync
cdr_serialize(
  const ugv_sensor_sync::msg::UwbRange & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ugv_sensor_sync
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ugv_sensor_sync::msg::UwbRange & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ugv_sensor_sync
get_serialized_size(
  const ugv_sensor_sync::msg::UwbRange & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ugv_sensor_sync
max_serialized_size_UwbRange(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ugv_sensor_sync

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ugv_sensor_sync
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ugv_sensor_sync, msg, UwbRange)();

#ifdef __cplusplus
}
#endif

#endif  // UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
