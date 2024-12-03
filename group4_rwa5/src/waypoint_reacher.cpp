#include "waypoint_reacher.hpp"


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
    default:
        allowable_tolerance_ = 0.1;  // Default to SMALL if unknown
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
        return;
    }

    // Time calculation to determine elapsed time between control loop executions
    auto current_time = std::chrono::steady_clock::now();
    double dt = 0.1;  // Default value for the first run
    if (previous_time_.time_since_epoch().count() != 0)
    {
        dt = std::chrono::duration<double>(current_time - previous_time_).count();
    }
    previous_time_ = current_time;

    // Assume we have the robot's current position (for simulation purposes, starting at origin)
    static double current_x = 0.0;
    static double current_y = 0.0;
    static double current_theta = 0.0;

    double error_x = current_waypoint_.waypoint.x - current_x;
    double error_y = current_waypoint_.waypoint.y - current_y;

    // Calculate distance and angle to the target waypoint
    double distance_to_waypoint = std::sqrt(error_x * error_x + error_y * error_y);
    double target_angle = std::atan2(error_y, error_x);
    double angle_error = target_angle - current_theta;

    // Proportional control gains
    double k_p_linear = 0.3;  // Lowered gains for smoother, more realistic movement
    double k_p_angular = 1.0;

    geometry_msgs::msg::Twist velocity_command;

    // Ensure angle is between -π and π
    if (angle_error > M_PI)
        angle_error -= 2 * M_PI;
    if (angle_error < -M_PI)
        angle_error += 2 * M_PI;

    // Control logic: move towards the waypoint if it's not reached
    if (distance_to_waypoint > allowable_tolerance_)
    {
        // Set linear and angular velocity to move towards waypoint
        velocity_command.linear.x = std::min(k_p_linear * distance_to_waypoint, 0.5); // Limiting linear speed for more realistic movement
        velocity_command.angular.z = std::min(k_p_angular * angle_error, 1.0);        // Limiting angular speed for stability

        RCLCPP_INFO(this->get_logger(), "Moving towards waypoint: current position x=%f, y=%f, theta=%f", current_x, current_y, current_theta);
        RCLCPP_INFO(this->get_logger(), "Distance to waypoint: %f, Target angle error: %f", distance_to_waypoint, angle_error);
    }
    else
    {
        // At waypoint, now rotate to the desired theta
        double theta_error = current_waypoint_.waypoint.theta - current_theta;

        if (std::abs(theta_error) > 0.05)  // Allow small threshold for angle error
        {
            velocity_command.angular.z = std::min(k_p_angular * theta_error, 1.0);  // Rotate to align with target angle
            RCLCPP_INFO(this->get_logger(), "Rotating to target theta: current theta=%f, Target theta=%f, Angle error=%f", current_theta, current_waypoint_.waypoint.theta, theta_error);
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

                // Stop the robot by setting all velocities to zero
                velocity_command.linear.x = 0.0;
                velocity_command.angular.z = 0.0;

                RCLCPP_INFO(this->get_logger(), "Waypoint %d reached. Publishing to /next_waypoint", waypoint_reached_count_);
            }
        }
    }

    // Publish the velocity command to control the robot
    velocity_publisher_->publish(velocity_command);

    // Simulate updating the robot's current position (for illustration purposes)
    current_x += velocity_command.linear.x * dt * cos(current_theta);    // Update position based on velocity
    current_y += velocity_command.linear.x * dt * sin(current_theta);    // Update position based on velocity
    current_theta += velocity_command.angular.z * dt;                    // Update angle based on angular velocity

    // Ensure theta remains in the range -π to π for better error handling
    if (current_theta > M_PI)
        current_theta -= 2 * M_PI;
    if (current_theta < -M_PI)
        current_theta += 2 * M_PI;
}