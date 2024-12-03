#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include <array>
#include <std_msgs/msg/bool.hpp>
#include "nav_msgs/msg/odometry.hpp"


class WaypointPublisher : public rclcpp::Node
{
public:
    // Constructor
    WaypointPublisher()
    : Node("waypoint_publisher"), index_(0)
    {
        // Initialize the waypoints
        waypoints_[0].waypoint.x = 0.0;
        waypoints_[0].waypoint.y = 0.0;
        waypoints_[0].waypoint.theta = 0.0;
        waypoints_[0].tolerance = bot_waypoint_msgs::msg::BotWaypoint::SMALL;

        waypoints_[1].waypoint.x = 4.0;
        waypoints_[1].waypoint.y = 4.0;
        waypoints_[1].waypoint.theta = 1.57;
        waypoints_[1].tolerance = bot_waypoint_msgs::msg::BotWaypoint::SMALL;

        // waypoints_[2].waypoint.x = 4.0;
        // waypoints_[2].waypoint.y = -4.0;
        // waypoints_[2].waypoint.theta = 3.14;
        // waypoints_[2].tolerance = bot_waypoint_msgs::msg::BotWaypoint::MEDIUM;

        // waypoints_[3].waypoint.x = -4.0;
        // waypoints_[3].waypoint.y = 4.0;
        // waypoints_[3].waypoint.theta = -3.14;
        // waypoints_[3].tolerance = bot_waypoint_msgs::msg::BotWaypoint::LARGE;

        // Publisher setup for the waypoint topic
        publisher_ = this->create_publisher<bot_waypoint_msgs::msg::BotWaypoint>("/bot_waypoint", 10);
        publishWaypoint();
        // Subscriber setup for the next_waypoint topic
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/next_waypoint", 10, std::bind(&WaypointPublisher::nextWaypointCallback, this, std::placeholders::_1));

    }

private:
    // Method to publish the waypoint
    void publishWaypoint();

    // Callback function when the next_waypoint signal is received
    void nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    std::array<bot_waypoint_msgs::msg::BotWaypoint, 4> waypoints_;
    unsigned int index_{0};  // Index to keep track of the current waypoint
};