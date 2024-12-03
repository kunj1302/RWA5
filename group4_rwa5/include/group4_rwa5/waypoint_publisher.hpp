#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include <array>
#include <std_msgs/msg/bool.hpp>

class WaypointPublisher : public rclcpp::Node
{
public:
    // Constructor
    WaypointPublisher();

private:
    // Method to publish the waypoint
    void publishWaypoint();

    // Callback function when the next_waypoint signal is received
    void nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    std::array<bot_waypoint_msgs::msg::BotWaypoint, 3> waypoints_;
    unsigned int index_;  // Index to keep track of the current waypoint
};