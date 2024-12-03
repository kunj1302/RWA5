#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "std_msgs/msg/bool.hpp"
#include <array>

class WaypointPublisher : public rclcpp::Node {
public:
    WaypointPublisher();

private:
    // Publish the current waypoint
    void publishWaypoint();

    // Callback to handle the next waypoint signal
    void nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr next_waypoint_subscription_;

    std::array<bot_waypoint_msgs::msg::BotWaypoint, 4> waypoints_; // Array of waypoints
    unsigned int current_index_{0}; // Index to keep track of the current waypoint
};
