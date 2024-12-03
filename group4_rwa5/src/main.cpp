#include <rclcpp/rclcpp.hpp>
#include "waypoint_publisher.hpp"
// Include any additional node headers you may have

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create shared pointers for both nodes
    auto waypoint_publisher_node = std::make_shared<WaypointPublisher>();
    // Create other nodes here
    // auto another_node = std::make_shared<AnotherNode>();

    // Multi-threaded executor to run multiple nodes concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(waypoint_publisher_node);
    // Add other nodes to the executor
    // executor.add_node(another_node);

    // Spin the executor
    executor.spin();

    rclcpp::shutdown();
}