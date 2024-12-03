#include "waypoint_reacher.hpp"

WaypointReacher::WaypointReacher()
    : Node("waypoint_reacher"), waypoint_reached_count_(0), is_waypoint_reached_(false) {
    // Subscriber for waypoints
    waypoint_subscription_ = this->create_subscription<bot_waypoint_msgs::msg::BotWaypoint>(
        "/bot_waypoint", 10, std::bind(&WaypointReacher::waypointCallback, this, std::placeholders::_1));

    // Subscriber for odometry data
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointReacher::odomCallback, this, std::placeholders::_1));

    // Publisher for signaling next waypoint
    next_waypoint_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/next_waypoint", 10);

    // Publisher for velocity commands
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for control loop
    controller_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&WaypointReacher::controlLoop, this));
}

void WaypointReacher::waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg) {
    current_waypoint_ = *msg;

    // Map tolerance values to numerical thresholds
    switch (current_waypoint_.tolerance) {
    case bot_waypoint_msgs::msg::BotWaypoint::SMALL:
        allowable_tolerance_ = 0.1; // 0.1 meters
        break;
    case bot_waypoint_msgs::msg::BotWaypoint::MEDIUM:
        allowable_tolerance_ = 0.2; // 0.2 meters
        break;
    case bot_waypoint_msgs::msg::BotWaypoint::LARGE:
        allowable_tolerance_ = 0.3; // 0.3 meters
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "Unknown tolerance value. Defaulting to 0.1 meters.");
        allowable_tolerance_ = 0.1; // Fallback value
        break;
    }

    is_waypoint_reached_ = false; // Reset flag for new waypoint
    RCLCPP_INFO(this->get_logger(), "Received new waypoint: x=%f, y=%f, theta=%f, tolerance=%f",
                current_waypoint_.waypoint.x, current_waypoint_.waypoint.y, current_waypoint_.waypoint.theta, allowable_tolerance_);
}

void WaypointReacher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update current position and orientation from odometry
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, current_theta_);
}

void WaypointReacher::controlLoop() {
    if (is_waypoint_reached_ || waypoint_reached_count_ >= 4) {
        return;
    }

    double error_x = current_waypoint_.waypoint.x - current_x_;
    double error_y = current_waypoint_.waypoint.y - current_y_;

    double distance_to_waypoint = std::sqrt(error_x * error_x + error_y * error_y);
    double target_angle = std::atan2(error_y, error_x);
    double angle_error = normalizeAngle(target_angle - current_theta_);

    geometry_msgs::msg::Twist velocity_command;

    // Proportional gains
    double k_p_linear = 0.3;
    double k_p_angular = 1.0;

    if (distance_to_waypoint > allowable_tolerance_) {
        // Move towards the waypoint
        velocity_command.linear.x = std::min(k_p_linear * distance_to_waypoint, 0.5);
        velocity_command.angular.z = std::min(k_p_angular * angle_error, 1.0);
    } else {
        double theta_error = normalizeAngle(current_waypoint_.waypoint.theta - current_theta_);
        if (std::abs(theta_error) > 0.05) {
            // Rotate to align with the waypoint's orientation
            velocity_command.angular.z = std::min(k_p_angular * theta_error, 1.0);
        } else if (!is_waypoint_reached_) {
            // Signal waypoint reached
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            next_waypoint_publisher_->publish(msg);
            waypoint_reached_count_++;
            is_waypoint_reached_ = true;
        }
    }

    velocity_publisher_->publish(velocity_command);
}

double WaypointReacher::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
