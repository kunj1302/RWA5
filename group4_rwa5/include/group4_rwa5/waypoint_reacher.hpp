#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>

class WaypointReacher : public rclcpp::Node
{
public:
    // Constructor
    WaypointReacher();

private:
    // Callback when a waypoint is received
    void waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg);

    // Proportional controller to guide the robot to the waypoint
    void controlLoop();

    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_waypoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr controller_timer_;

    bot_waypoint_msgs::msg::BotWaypoint current_waypoint_;
    int waypoint_reached_count_;  // Count of how many waypoints have been reached
    bool is_waypoint_reached_;    // Flag to ensure waypoint is published once
    std::chrono::steady_clock::time_point previous_time_{};
};