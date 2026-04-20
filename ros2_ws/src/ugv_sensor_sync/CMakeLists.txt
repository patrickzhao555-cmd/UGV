cmake_minimum_required(VERSION 3.8)
project(ugv_sensor_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UwbRange.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(clock_mapper STATIC src/clock_mapper.cpp)
ament_target_dependencies(clock_mapper rclcpp)

add_executable(uwb_time_bridge src/uwb_time_bridge.cpp)
ament_target_dependencies(uwb_time_bridge
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
target_link_libraries(uwb_time_bridge clock_mapper)
rosidl_target_interfaces(uwb_time_bridge ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(scan_time_corrector src/scan_time_corrector.cpp)
ament_target_dependencies(scan_time_corrector
  rclcpp
  sensor_msgs
)

add_executable(planner_sync_node src/planner_sync_node.cpp)
ament_target_dependencies(planner_sync_node
  rclcpp
  sensor_msgs
  message_filters
)
rosidl_target_interfaces(planner_sync_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  uwb_time_bridge
  scan_time_corrector
  planner_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()