#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class WaypointReacher : public rclcpp::Node {
public:
    WaypointReacher();

private:
    // Callback to process received waypoint messages
    void waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg);

    // Odometry callback to update the robot's position and orientation
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Control loop to guide the robot to the waypoint
    void controlLoop();

    // Publish velocity commands
    void publishVelocity(double linear, double angular);

    // Normalize angles to the range [-π, π]
    double normalizeAngle(double angle);

    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_waypoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr controller_timer_;

    bot_waypoint_msgs::msg::BotWaypoint current_waypoint_;
    int waypoint_reached_count_{0};
    bool is_waypoint_reached_{false};
    double allowable_tolerance_{0.1}; // Dynamic tolerance based on waypoint tolerance
    double current_x_{0.0}, current_y_{0.0}, current_theta_{0.0};
    double roll_{0.0}, pitch_{0.0};
};
