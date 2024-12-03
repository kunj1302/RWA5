#include "waypoint_reacher.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

// Callback when a waypoint is received
void WaypointReacher::waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg)
{
    current_waypoint_ = *msg;

    // Convert the tolerance value into a numerical distance based on SMALL, MEDIUM, LARGE
    switch (current_waypoint_.tolerance)
    {
    case bot_waypoint_msgs::msg::BotWaypoint::SMALL:
        allowable_tolerance_ = 0.1;  // 0.1 meters for SMALL
        break;
    case bot_waypoint_msgs::msg::BotWaypoint::MEDIUM:
        allowable_tolerance_ = 0.2;  // 0.2 meters for MEDIUM
        break;
    case bot_waypoint_msgs::msg::BotWaypoint::LARGE:
        allowable_tolerance_ = 0.3;  // 0.3 meters for LARGE
        break;
    }

    is_waypoint_reached_ = false;  // Reset the flag since a new waypoint has been received
    RCLCPP_INFO(this->get_logger(), "Received new waypoint: x=%f, y=%f, theta=%f, tolerance=%f",
                msg->waypoint.x, msg->waypoint.y, msg->waypoint.theta, allowable_tolerance_);
}

// Proportional controller to guide the robot to the waypoint
void WaypointReacher::controlLoop()
{
    if (is_waypoint_reached_ || waypoint_reached_count_ >= 3)
    {
        return; // Stop control loop if the waypoint is reached or after 3 attempts
    }

    // Get current position (using /odom or /pose topic)
    // Let's assume you have the position in `current_x_`, `current_y_` and orientation `current_theta_`
    double dx = current_waypoint_.waypoint.x - current_x_; // X distance to target
    double dy = current_waypoint_.waypoint.y - current_y_; // Y distance to target
    double distance = std::sqrt(dx * dx + dy * dy); // Euclidean distance

    
    // Check if the robot has reached the target within tolerance
    if (distance <= allowable_tolerance_)
    {
        is_waypoint_reached_ = true; // Mark waypoint as reached
        waypoint_reached_count_++; // Increment the waypoint reached counter

        // Publish next waypoint signal
        std_msgs::msg::Bool next_waypoint_msg;
        next_waypoint_msg.data = true;
        next_waypoint_publisher_->publish(next_waypoint_msg);
        RCLCPP_INFO(this->get_logger(), "Waypoint reached! Publishing next waypoint signal.");
        return; // Stop the control loop once the waypoint is reached
    }

    // Control Logic: You can use a proportional controller for simple movement
    double angle_to_target = std::atan2(dy, dx); // Angle towards the target
    double angle_error = angle_to_target - current_theta_; // Error between robot orientation and target angle

    // Ensure the angle error is between -pi and pi
    while (angle_error > M_PI)
        angle_error -= 2 * M_PI;
    while (angle_error < -M_PI)
        angle_error += 2 * M_PI;

    // Simple proportional control for linear and angular velocity
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.5 * distance; // Move proportional to distance
    cmd_vel_msg.angular.z = 1.0 * angle_error; // Rotate proportional to angle error

    // Publish the cmd_vel message to control the robot
    velocity_publisher_->publish(cmd_vel_msg);

    RCLCPP_INFO(this->get_logger(), "Moving to waypoint: x=%f, y=%f, theta=%f, distance=%f",
                current_waypoint_.waypoint.x, current_waypoint_.waypoint.y, current_waypoint_.waypoint.theta, distance);
}