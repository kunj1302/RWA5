#include "waypoint_publisher.hpp"

// Method to publish the waypoint
void WaypointPublisher::publishWaypoint()
{
    if (index_ < waypoints_.size())  // Ensure index is within bounds
    {
        current_waypoint_ = waypoints_[index_];  // Get the current waypoint
        publisher_->publish(current_waypoint_);
        is_published_ = true;  // Set the flag to indicate the waypoint is published
        RCLCPP_INFO(this->get_logger(), "Published waypoint %d: x=%f, y=%f, theta=%f",
                    index_, current_waypoint_.waypoint.x, current_waypoint_.waypoint.y, current_waypoint_.waypoint.theta);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "All waypoints have been published.");
    }
}

// Callback function when the next_waypoint signal is received
void WaypointPublisher::onNextWaypointReceived(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && is_published_)
    {
        // Move to the next waypoint
        index_++;
        is_published_ = false;  // Reset the flag to allow the next waypoint to be published

        // Publish the next waypoint if available
        if (index_ < waypoints_.size())
        {
            publishWaypoint();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached, no more waypoints to publish.");
        }
    }
}