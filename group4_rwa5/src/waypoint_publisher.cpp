#include "waypoint_publisher.hpp"

// Constructor
WaypointPublisher::WaypointPublisher()
    : Node("waypoint_publisher"), index_(0)
{
    // Initialize the waypoints
    waypoints_[0].waypoint.x = 0.0;
    waypoints_[0].waypoint.y = 0.0;
    waypoints_[0].waypoint.theta = 0.0;
    waypoints_[0].tolerance = bot_waypoint_msgs::msg::BotWaypoint::SMALL;

    waypoints_[1].waypoint.x = 4.0;
    waypoints_[1].waypoint.y = 4.0;
    waypoints_[1].waypoint.theta = 1.57;
    waypoints_[1].tolerance = bot_waypoint_msgs::msg::BotWaypoint::SMALL;

    waypoints_[2].waypoint.x = 4.0;
    waypoints_[2].waypoint.y = -4.0;
    waypoints_[2].waypoint.theta = 3.14;
    waypoints_[2].tolerance = bot_waypoint_msgs::msg::BotWaypoint::MEDIUM;

    waypoints_[3].waypoint.x = -4.0;
    waypoints_[3].waypoint.y = 4.0;
    waypoints_[3].waypoint.theta = -3.14;
    waypoints_[3].tolerance = bot_waypoint_msgs::msg::BotWaypoint::LARGE;

    // Publisher setup for the waypoint topic
    publisher_ = this->create_publisher<bot_waypoint_msgs::msg::BotWaypoint>("/bot_waypoint", 10);
    publishWaypoint();
    // Subscriber setup for the next_waypoint topic
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/next_waypoint", 10, std::bind(&WaypointPublisher::nextWaypointCallback, this, std::placeholders::_1));

}

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