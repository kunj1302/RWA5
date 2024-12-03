#include "waypoint_reacher.hpp"

void WaypointReacher::waypoint_callback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg) {
    // Update goal with received waypoint
    goal_x_ = msg->waypoint.x;
    goal_y_ = msg->waypoint.y;
    goal_theta_ = msg->waypoint.theta;

    RCLCPP_INFO(this->get_logger(), "New waypoint received: x=%.2f, y=%.2f, theta=%.2f",
                goal_x_, goal_y_, goal_theta_);
}

void WaypointReacher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract current position
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Extract current orientation as yaw (theta)
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, current_theta_);

    control_loop();
}

void WaypointReacher::control_loop() {
    // Calculate distance and angular errors
    double distance_error = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    double desired_angle = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
    double angle_error = normalize_angle(desired_angle - current_theta_);
    double final_orientation_error = normalize_angle(goal_theta_ - current_theta_);

    // Navigate to goal position
    if (distance_error > epsilon_) {
        double linear_velocity = kp_distance_ * distance_error;
        double angular_velocity = kp_angle_ * angle_error;

        publish_velocity(linear_velocity, angular_velocity);
    }
    // Adjust final orientation
    else if (std::abs(final_orientation_error) > epsilon_theta_) {
        double angular_velocity = kp_angle_ * final_orientation_error;

        publish_velocity(0.0, angular_velocity);
    }
    // Goal reached
    else {
        publish_velocity(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Goal reached: x=%.2f, y=%.2f, theta=%.2f",
                    goal_x_, goal_y_, goal_theta_);
    }
}

void WaypointReacher::publish_velocity(double linear, double angular) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    velocity_publisher_->publish(msg);
}


double WaypointReacher::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}