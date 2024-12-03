cmake_minimum_required(VERSION 3.8)
project(bot_waypoint_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/BotWaypoint.msg")


rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files}
  DEPENDENCIES example_interfaces geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
