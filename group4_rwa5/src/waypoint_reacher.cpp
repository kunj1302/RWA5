#include "waypoint_reacher.hpp"
#include <cmath>


// Callback when a waypoint is received
void WaypointReacher::waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg)
{
    current_waypoint_ = *msg;
    waypoint_received_ = true;
    is_waypoint_reached_ = false;  // Reset the flag since a new waypoint has been received
    RCLCPP_INFO(this->get_logger(), "Received new waypoint: x=%f, y=%f, theta=%f", msg->waypoint.x, msg->waypoint.y, msg->waypoint.theta);
}

// Callback when odometry data is received
void WaypointReacher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Extracting the orientation (theta) from the quaternion
    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    current_theta_ = std::atan2(siny_cosp, cosy_cosp);

    // Debug log for current position and orientation
    RCLCPP_DEBUG(this->get_logger(), "Current position: x=%f, y=%f, theta=%f", current_x_, current_y_, current_theta_);
}

// Proportional controller to guide the robot to the waypoint
void WaypointReacher::controlLoop()
{
    if (!waypoint_received_ || is_waypoint_reached_ || waypoint_reached_count_ >= 3)
    {
        return;
    }

    double error_x = current_waypoint_.waypoint.x - current_x_;
    double error_y = current_waypoint_.waypoint.y - current_y_;

    // Calculate distance and angle to the target waypoint
    double distance_to_waypoint = std::sqrt(error_x * error_x + error_y * error_y);
    double target_angle = std::atan2(error_y, error_x);
    double angle_error = target_angle - current_theta_;

    // Proportional control gains
    double k_p_linear = 0.5;
    double k_p_angular = 1.0;

    geometry_msgs::msg::Twist velocity_command;

    // Control logic: move towards the waypoint if it's not reached
    if (distance_to_waypoint > current_waypoint_.tolerance)
    {
        // Set linear and angular velocity to move towards waypoint
        velocity_command.linear.x = k_p_linear * distance_to_waypoint;
        velocity_command.angular.z = k_p_angular * angle_error;

        RCLCPP_INFO(this->get_logger(), "Moving towards waypoint: x=%f, y=%f, theta=%f", current_x_, current_y_, current_theta_);
        RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f, Target angle error: %f", distance_to_waypoint, angle_error);
    }
    else
    {
        // At waypoint, now rotate to the desired theta
        double theta_error = current_waypoint_.waypoint.theta - current_theta_;

        if (std::abs(theta_error) > 0.05)  // Allow small threshold for angle error
        {
            velocity_command.angular.z = k_p_angular * theta_error;  // Rotate to align with target angle
            RCLCPP_INFO(this->get_logger(), "Rotating to target theta: theta=%f, Current angle error: %f", current_theta_, theta_error);
        }
        else
        {
            // Waypoint and theta reached, stop moving and publish true to next_waypoint
            if (!is_waypoint_reached_)
            {
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                next_waypoint_publisher_->publish(msg);
                waypoint_reached_count_++;
                is_waypoint_reached_ = true;
                waypoint_received_ = false;  // Reset to wait for next waypoint

                // Stop the robot by setting all velocities to zero
                velocity_command.linear.x = 0.0;
                velocity_command.angular.z = 0.0;

                RCLCPP_INFO(this->get_logger(), "Waypoint %d reached. Publishing to /next_waypoint", waypoint_reached_count_);
            }
        }
    }

    // Publish the velocity command to control the robot
    velocity_publisher_->publish(velocity_command);
    std::cout<<"Possibel\n";

    // Debug log for current position and command
    RCLCPP_DEBUG(this->get_logger(), "Velocity command: linear_x=%f, angular_z=%f", velocity_command.linear.x, velocity_command.angular.z);
}