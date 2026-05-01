cmake_minimum_required(VERSION 3.8)
project(ugv_sensor_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderTicksStamped.msg"
  "msg/NavSensorFrame.msg"
  "msg/SyncedSensorPacket.msg"
  "msg/UwbRange.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs builtin_interfaces
)

ament_python_install_package(ugv_sensor_sync_nodes)

install(PROGRAMS
  scripts/bench_goal_node
  scripts/debug_status_node
  scripts/fusion_node
  scripts/lidar_sync_node
  scripts/mock_field_map_node
  scripts/uwb_node
  scripts/zed_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  README
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
