#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include <array>
#include <std_msgs/msg/bool.hpp>

class WaypointPublisher : public rclcpp::Node
{
public:
    // Constructor
    WaypointPublisher()
        : Node("waypoint_publisher"), index_(0), is_published_(false)
    {
        // Initialize the waypoints with uniform initialization
        bot_status_msg_ = bot_waypoint_msgs::msg::BotWaypoint();
        waypoints_[0].waypoint.x = 4.0;
        waypoints_[0].waypoint.y = 4.0;
        waypoints_[0].waypoint.theta = 1.57;
        waypoints_[0].tolerance = bot_waypoint_msgs::msg::BotWaypoint::SMALL;

        // Publisher setup for the waypoint topic
        publisher_ = this->create_publisher<bot_waypoint_msgs::msg::BotWaypoint>("/bot_waypoint", 10);

        // Subscriber setup for the next_waypoint topic
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/next_waypoint", 10, std::bind(&WaypointPublisher::nextWaypointCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WaypointPublisher::publish_bot_status, this));
        // Publish the first waypoint immediately
        publishWaypoint();
    }



private:
    // Method to publish the waypoint
    void publishWaypoint();

    void publish_bot_status();
    // Callback function when the next_waypoint signal is received
    void nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<bot_waypoint_msgs::msg::BotWaypoint, 3> waypoints_; 
    bot_waypoint_msgs::msg::BotWaypoint bot_status_msg_; // Array to store waypoints
    unsigned int index_;  // Index to keep track of the current waypoint
    bool is_published_;  // Flag to ensure each waypoint is published only once
};