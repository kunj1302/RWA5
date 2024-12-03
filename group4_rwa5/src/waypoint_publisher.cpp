#include "waypoint_publisher.hpp"


// Method to publish the waypoint
void WaypointPublisher::publishWaypoint()
{
    if (index_ < waypoints_.size()) {
        publisher_->publish(waypoints_[index_]);
        is_published_ = true;
    }
}

// Method to periodically publish bot status (current waypoint info)
void WaypointPublisher::publish_bot_status()
{
    if (index_ < waypoints_.size()) {
        bot_status_msg_.waypoint.x = waypoints_[index_].waypoint.x;
        bot_status_msg_.waypoint.y = waypoints_[index_].waypoint.y;
        bot_status_msg_.waypoint.theta = waypoints_[index_].waypoint.theta;
        bot_status_msg_.tolerance = waypoints_[index_].tolerance;

        // Publish the bot status message
        RCLCPP_INFO(this->get_logger(), "Publishing bot status: (x: %.2f, y: %.2f, theta: %.2f, tolerance: %d)",
                    bot_status_msg_.waypoint.x, bot_status_msg_.waypoint.y,
                    bot_status_msg_.waypoint.theta, bot_status_msg_.tolerance);
    }
}

// Callback function when the next_waypoint signal is received
void WaypointPublisher::nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !is_published_ && index_ < waypoints_.size()) {
        // Increment the index to publish the next waypoint
        ++index_;
        publishWaypoint();
    }
}