cmake_minimum_required(VERSION 3.8)
project(group4_rwa5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bot_waypoint_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Include directories for custom message
include_directories(include/group4_rwa5 ${bot_waypoint_msgs_INCLUDE_DIRS})

# Add executable
add_executable(waypoint src/main.cpp src/waypoint_publisher.cpp src/waypoint_reacher.cpp)

# Link libraries
target_link_libraries(waypoint
  ${rclcpp_LIBRARIES}
)

# Set target dependencies (CMake will manage message header files)
ament_target_dependencies(waypoint rclcpp bot_waypoint_msgs nav_msgs)

# Install the executable
install(TARGETS waypoint DESTINATION lib/${PROJECT_NAME})

ament_package()