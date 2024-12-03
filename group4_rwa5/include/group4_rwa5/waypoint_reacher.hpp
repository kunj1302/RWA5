#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>
#include "nav_msgs/msg/odometry.hpp"


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

        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointReacher::odomCallback, this, std::placeholders::_1));

        // Timer to control the robot towards the waypoint at fixed intervals
        controller_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WaypointReacher::controlLoop, this));
    }

private:
    // Callback when a waypoint is received
    void waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract the position (x, y) from the odometry message
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Extract the orientation (theta) from the quaternion
        double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                                msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                        msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }
    // Proportional controller to guide the robot to the waypoint
    void controlLoop();

    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_waypoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;
    bot_waypoint_msgs::msg::BotWaypoint current_waypoint_;
    int waypoint_reached_count_{0};  // Count of how many waypoints have been reached
    bool is_waypoint_reached_{false};
    float allowable_tolerance_{0};  
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

    std::chrono::steady_clock::time_point previous_time_;
};