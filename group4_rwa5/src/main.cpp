#include <rclcpp/rclcpp.hpp>
#include "waypoint_publisher.hpp"
#include "waypoint_reacher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto waypoint_publisher_node = std::make_shared<WaypointPublisher>();
    auto waypoint_reacher_node = std::make_shared<WaypointReacher>();
  
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(waypoint_publisher_node);
    executor.add_node(waypoint_reacher_node);

    executor.spin();

    rclcpp::shutdown();
}