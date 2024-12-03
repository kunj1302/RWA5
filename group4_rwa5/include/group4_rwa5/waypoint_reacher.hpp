#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // Use PoseStamped instead of Odometry

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

        // Subscriber for /pose to receive current position and orientation of the robot
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_topic", 10, std::bind(&WaypointReacher::poseCallback, this, std::placeholders::_1));

        // Publisher for /next_waypoint to signal when a waypoint is reached
        next_waypoint_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/next_waypoint", 10);

        // Publisher for /cmd_vel to control TurtleBot movement
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer to control the robot towards the waypoint at fixed intervals
        controller_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&WaypointReacher::controlLoop, this));
    }

private:
    // Callback when a waypoint is received
    void waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg);

    // Callback when pose data is received (using PoseStamped instead of Odometry)
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Proportional controller to guide the robot to the waypoint
    void controlLoop();

    // Subscription and publisher members
    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_; // Subscription for PoseStamped
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_waypoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr controller_timer_;

    bot_waypoint_msgs::msg::BotWaypoint current_waypoint_;  // The current waypoint to reach

    // Current position and orientation of the robot
    double current_x_{0.0};    
    double current_y_{0.0};    
    double current_theta_{0.0}; 

    // Flags and counters for waypoint management
    bool waypoint_received_{false};  
    bool is_waypoint_reached_;       
    int waypoint_reached_count_;     
};