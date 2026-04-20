// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ugv_sensor_sync:msg/UwbRange.idl
// generated code does not contain a copyright notice

#ifndef UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__BUILDER_HPP_
#define UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ugv_sensor_sync/msg/detail/uwb_range__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ugv_sensor_sync
{

namespace msg
{

namespace builder
{

class Init_UwbRange_anchor_id
{
public:
  explicit Init_UwbRange_anchor_id(::ugv_sensor_sync::msg::UwbRange & msg)
  : msg_(msg)
  {}
  ::ugv_sensor_sync::msg::UwbRange anchor_id(::ugv_sensor_sync::msg::UwbRange::_anchor_id_type arg)
  {
    msg_.anchor_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_sensor_sync::msg::UwbRange msg_;
};

class Init_UwbRange_range_stddev_m
{
public:
  explicit Init_UwbRange_range_stddev_m(::ugv_sensor_sync::msg::UwbRange & msg)
  : msg_(msg)
  {}
  Init_UwbRange_anchor_id range_stddev_m(::ugv_sensor_sync::msg::UwbRange::_range_stddev_m_type arg)
  {
    msg_.range_stddev_m = std::move(arg);
    return Init_UwbRange_anchor_id(msg_);
  }

private:
  ::ugv_sensor_sync::msg::UwbRange msg_;
};

class Init_UwbRange_range_m
{
public:
  explicit Init_UwbRange_range_m(::ugv_sensor_sync::msg::UwbRange & msg)
  : msg_(msg)
  {}
  Init_UwbRange_range_stddev_m range_m(::ugv_sensor_sync::msg::UwbRange::_range_m_type arg)
  {
    msg_.range_m = std::move(arg);
    return Init_UwbRange_range_stddev_m(msg_);
  }

private:
  ::ugv_sensor_sync::msg::UwbRange msg_;
};

class Init_UwbRange_esp32_time_us
{
public:
  explicit Init_UwbRange_esp32_time_us(::ugv_sensor_sync::msg::UwbRange & msg)
  : msg_(msg)
  {}
  Init_UwbRange_range_m esp32_time_us(::ugv_sensor_sync::msg::UwbRange::_esp32_time_us_type arg)
  {
    msg_.esp32_time_us = std::move(arg);
    return Init_UwbRange_range_m(msg_);
  }

private:
  ::ugv_sensor_sync::msg::UwbRange msg_;
};

class Init_UwbRange_header
{
public:
  Init_UwbRange_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UwbRange_esp32_time_us header(::ugv_sensor_sync::msg::UwbRange::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UwbRange_esp32_time_us(msg_);
  }

private:
  ::ugv_sensor_sync::msg::UwbRange msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_sensor_sync::msg::UwbRange>()
{
  return ugv_sensor_sync::msg::builder::Init_UwbRange_header();
}

}  // namespace ugv_sensor_sync

#endif  // UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__BUILDER_HPP_
