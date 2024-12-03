#include "waypoint_publisher.hpp"

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
