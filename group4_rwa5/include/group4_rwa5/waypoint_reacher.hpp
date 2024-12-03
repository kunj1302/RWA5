#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>


/**
 * @class WaypointReacher 
 * @brief A ROS2 node that makes the turtlebot navigate through a sequence of waypoints 
 */ 
class WaypointReacher : public rclcpp::Node {
public:
    WaypointReacher()
        : Node("waypoint_reacher"),
          waypoint_reached_count_{0},
          is_waypoint_reached_{false},
          allowable_tolerance_{default_tolerance_},
          current_x_{0.0}, current_y_{0.0}, current_theta_{0.0},
          roll_{0.0}, pitch_{0.0} {
        // Subscriber for waypoint data    
        waypoint_subscription_ = this->create_subscription<bot_waypoint_msgs::msg::BotWaypoint>(
            "/bot_waypoint", 10, std::bind(&WaypointReacher::waypointCallback, this, std::placeholders::_1));

        // Subscriber for odometry data
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WaypointReacher::odomCallback, this, std::placeholders::_1));

        // Publisher for next waypoint signal
        next_waypoint_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/next_waypoint", 10);

        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer for control loop
        controller_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&WaypointReacher::controlLoop, this));
    }

private:

    static constexpr double default_tolerance_ = 0.1; /**< Default tolerance of 0.1 metre */

    /**
     * @brief Callback function for processing the waypoint messages received 
     * 
     * This function updates the current waypoint and sets the tolerance level
     * based on the waypoint's requirement of SMALL MEDIUM or LARGE.
     *
     * @param msg Shared pointer for the waypoint message.
     */

    void waypointCallback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg);

    /**
     * @brief Callback function for processing the odometry values.
     *
     * Updates the robot's current position and orientation based on data received from the /odom topic
     *
     * @param msg Shared pointer to the odometry message received.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Executes the control logic for navigating to the goal.
     *
     * This function calculates the errors (distance and angle) and determines
     * the appropriate linear and angular velocity commands to move the robot
     * towards the goal.
     */
    void controlLoop();
    
    /**
     * @brief Normalizes an angle to the range [-π, π].
     *
     * Ensures that the input angle is within the valid range to avoid issues
     * caused by angle wraparound.
     *
     * @param angle Input angle in radians.
     * @return Normalized angle in radians within the range [-π, π].
     */
    double normalizeAngle(double angle);

    /**
     * @brief ROS 2 subscription for waypoint data.
     *
     * Subscribes to `bot_waypoint_msgs::msg::BotWaypoint` messages to obtain the waypoint
     * that the robot is supposed to move towards
     */
    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_subscription_;   
   
    /**
     * @brief ROS 2 subscription for odometry data.
     *
     * Subscribes to `nav_msgs::msg::Odometry` messages to obtain the robot's current
     * position and orientation.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;                   
    
    /**
     * @brief ROS 2 publisher for next waypoint.
     *
     * publishes a boolean message to indicate whether the robot is ready to proceed to the next waypoint
     *
     */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_waypoint_publisher_;                     

    // /**
    //  * @brief ROS 2 publisher for velocity commands.
    //  *
    //  * Publishes `geometry_msgs::msg::Twist` messages to control the robot's linear
    //  * and angular velocities.
    //  */
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;    

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    /**
     * @brief Timer for executing the control loop periodically.
     *
     * Triggers the control loop at a fixed interval to process the waypoints
     */
    rclcpp::TimerBase::SharedPtr controller_timer_;                                                
    /**
     * @brief The current waypoint being processed.
     *
     * Stores the position, orientation, and tolerance information for the active waypoint.
     */
    bot_waypoint_msgs::msg::BotWaypoint current_waypoint_;/**< Orientation components (roll and pitch). */


    int waypoint_reached_count_;                          /**< Counter to track the number of waypoints reached. */
    bool is_waypoint_reached_;                            /**< Flag indicating if the current waypoint has been reached. */
    double allowable_tolerance_;                          /**< Tolerance for considering a waypoint as reached. */

    /**
     * @brief Current state of the robot.
     *
     * These variables store the robot's current position, orientation, and
     * state-related information.
     */
    double current_x_;     /**< Current x-coordinate of the robot. */
    double current_y_;     /**< Current y-coordinate of the robot. */
    double current_theta_; /**< Current orientation (yaw) of the robot in radians. */
    double roll_;          /**< Current roll angle of the robot in radians (not actively used). */
    double pitch_;         /**< Current pitch angle of the robot in radians (not actively used). */
};

