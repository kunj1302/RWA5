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
    index_++;
}

// Callback function when the next_waypoint signal is received
void WaypointPublisher::nextWaypointCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        // index_++;
        if (index_ < waypoints_.size())
        {
            publishWaypoint();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "All waypoints have been published.");
        }
    }
}