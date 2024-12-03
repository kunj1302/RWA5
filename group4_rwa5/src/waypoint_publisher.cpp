#include "waypoint_publisher.hpp"

// Method to publish the waypoint
void WaypointPublisher::publishWaypoint()
{
    if (index_ < waypoints_.size())
    {
        RCLCPP_INFO(this->get_logger(), "Publishing waypoint %d: x=%f, y=%f, theta=%f", index_,
                    waypoints_[index_].waypoint.x, waypoints_[index_].waypoint.y, waypoints_[index_].waypoint.theta);
        publisher_->publish(waypoints_[index_]);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No more waypoints to publish. All waypoints have been sent.");
    }
}

// Callback function when the next_waypoint signal is received
void WaypointPublisher::nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        if (index_ < waypoints_.size())
        {
            publishWaypoint(); // Publish the current waypoint
            index_++;          // Increment the index after publishing
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "All waypoints have been published and processed.");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received a false signal. Waiting for a valid signal to publish next waypoint.");
    }
}
