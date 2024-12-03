#include "waypoint_publisher.hpp"

WaypointPublisher::WaypointPublisher()
    : Node("waypoint_publisher"), current_index_(0) {
    // Initialize waypoints with example data
    waypoints_[0].waypoint.x = 4.0;
    waypoints_[0].waypoint.y = 4.0;
    waypoints_[0].waypoint.theta = 1.57;
    waypoints_[0].tolerance = bot_waypoint_msgs::msg::BotWaypoint::SMALL;

    waypoints_[1].waypoint.x = 4.0;
    waypoints_[1].waypoint.y = -4.0;
    waypoints_[1].waypoint.theta = 3.14;
    waypoints_[1].tolerance = bot_waypoint_msgs::msg::BotWaypoint::MEDIUM;

    waypoints_[2].waypoint.x = -4.0;
    waypoints_[2].waypoint.y = 4.0;
    waypoints_[2].waypoint.theta = -3.14;
    waypoints_[2].tolerance = bot_waypoint_msgs::msg::BotWaypoint::LARGE;

    // Publisher for the waypoint topic
    waypoint_publisher_ = this->create_publisher<bot_waypoint_msgs::msg::BotWaypoint>(
        "/bot_waypoint", 10);

    // Subscriber for the next waypoint signal
    next_waypoint_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/next_waypoint", 10,
        std::bind(&WaypointPublisher::nextWaypointCallback, this, std::placeholders::_1));

    // Publish the first waypoint initially
    publishWaypoint();
}

void WaypointPublisher::publishWaypoint() {
    if (current_index_ < waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Publishing waypoint %d: x=%f, y=%f, theta=%f, tolerance=%d",
                    current_index_,
                    waypoints_[current_index_].waypoint.x,
                    waypoints_[current_index_].waypoint.y,
                    waypoints_[current_index_].waypoint.theta,
                    waypoints_[current_index_].tolerance);
        waypoint_publisher_->publish(waypoints_[current_index_]);
    } else {
        RCLCPP_WARN(this->get_logger(), "No more waypoints to publish.");
    }
}

void WaypointPublisher::nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        if (current_index_ < waypoints_.size()) {
            current_index_++;
            publishWaypoint();
        } else {
            RCLCPP_INFO(this->get_logger(), "All waypoints have been published.");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Received a false signal, waiting for the next valid signal.");
    }
}
