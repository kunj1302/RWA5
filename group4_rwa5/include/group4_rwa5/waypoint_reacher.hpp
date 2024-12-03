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
    WaypointReacher()
    : Node("waypoint_reacher"), waypoint_reached_count_(0), is_waypoint_reached_(false)
    {
        // Subscriber for /bot_waypoint to receive the current target waypoint
        waypoint_subscription_ = this->create_subscription<bot_waypoint_msgs::msg::BotWaypoint>(
            "/bot_waypoint", 10, std::bind(&WaypointReacher::waypointCallback, this, std::placeholders::_1));

        // Publisher for /next_waypoint to signal when a waypoint is reached
        next_waypoint_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/next_waypoint", 10);

        // Publisher for /cmd_vel to control TurtleBot movement
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer to control the robot towards the waypoint at fixed intervals
        controller_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WaypointReacher::controlLoop, this));
    }

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
    int waypoint_reached_count_{0};  // Count of how many waypoints have been reached
    bool is_waypoint_reached_{false};    // Flag to ensure waypoint is published once
    std::chrono::steady_clock::time_point previous_time_{};
};