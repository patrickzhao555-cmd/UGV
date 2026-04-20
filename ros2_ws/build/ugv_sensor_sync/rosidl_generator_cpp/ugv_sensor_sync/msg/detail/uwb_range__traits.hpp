// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ugv_sensor_sync:msg/UwbRange.idl
// generated code does not contain a copyright notice

#ifndef UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__TRAITS_HPP_
#define UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ugv_sensor_sync/msg/detail/uwb_range__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ugv_sensor_sync
{

namespace msg
{

inline void to_flow_style_yaml(
  const UwbRange & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: esp32_time_us
  {
    out << "esp32_time_us: ";
    rosidl_generator_traits::value_to_yaml(msg.esp32_time_us, out);
    out << ", ";
  }

  // member: range_m
  {
    out << "range_m: ";
    rosidl_generator_traits::value_to_yaml(msg.range_m, out);
    out << ", ";
  }

  // member: range_stddev_m
  {
    out << "range_stddev_m: ";
    rosidl_generator_traits::value_to_yaml(msg.range_stddev_m, out);
    out << ", ";
  }

  // member: anchor_id
  {
    out << "anchor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.anchor_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UwbRange & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: esp32_time_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "esp32_time_us: ";
    rosidl_generator_traits::value_to_yaml(msg.esp32_time_us, out);
    out << "\n";
  }

  // member: range_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range_m: ";
    rosidl_generator_traits::value_to_yaml(msg.range_m, out);
    out << "\n";
  }

  // member: range_stddev_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range_stddev_m: ";
    rosidl_generator_traits::value_to_yaml(msg.range_stddev_m, out);
    out << "\n";
  }

  // member: anchor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "anchor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.anchor_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UwbRange & msg, bool use_flow_style = false)
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

}  // namespace ugv_sensor_sync

namespace rosidl_generator_traits
{

[[deprecated("use ugv_sensor_sync::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ugv_sensor_sync::msg::UwbRange & msg,
  std::ostream & out, size_t indentation = 0)
{
  ugv_sensor_sync::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ugv_sensor_sync::msg::to_yaml() instead")]]
inline std::string to_yaml(const ugv_sensor_sync::msg::UwbRange & msg)
{
  return ugv_sensor_sync::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ugv_sensor_sync::msg::UwbRange>()
{
  return "ugv_sensor_sync::msg::UwbRange";
}

template<>
inline const char * name<ugv_sensor_sync::msg::UwbRange>()
{
  return "ugv_sensor_sync/msg/UwbRange";
}

template<>
struct has_fixed_size<ugv_sensor_sync::msg::UwbRange>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ugv_sensor_sync::msg::UwbRange>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ugv_sensor_sync::msg::UwbRange>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__TRAITS_HPP_
