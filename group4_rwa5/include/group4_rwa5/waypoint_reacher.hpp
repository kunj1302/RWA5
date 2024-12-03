#pragma once
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp" // Include custom waypoint message

class WaypointReacher : public rclcpp::Node {
   public:
    WaypointReacher()
        : Node("Waypoint_reacher_node") {
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for odometry data
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&WaypointReacher::odom_callback, this, std::placeholders::_1));

        // Waypoint Subscriber
        waypoint_subscription_ = this->create_subscription<bot_waypoint_msgs::msg::BotWaypoint>(
            "/bot_waypoint", 10,
            std::bind(&WaypointReacher::waypoint_callback, this, std::placeholders::_1));

        // Set the goal position and orientation
        goal_x_ = 0;
        goal_y_ = 0;
        goal_theta_ = M_PI / 2.0;  // Desired orientation (90 degrees)

        // Controller gains and tolerances
        kp_distance_ = 0.5;
        kp_angle_ = 2.0;
        epsilon_ = 0.1;         // Positional tolerance
        epsilon_theta_ = 0.05;  // Angular tolerance (radians)
        RCLCPP_INFO_STREAM(this->get_logger(), "== Proportional Controller Demo Node Started ==");
    }

   private:
    /**
     * @brief Callback function to process odometry messages.
     *
     * This function is called whenever an odometry message is received. It extracts
     * the current position and orientation of the robot from the message.
     *
     * @param msg Shared pointer to the incoming odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback function to process waypoint messages.
     *
     * Updates the goal position and orientation based on the received waypoint message.
     *
     * @param msg Shared pointer to the incoming waypoint message.
     */
    void waypoint_callback(const bot_waypoint_msgs::msg::BotWaypoint::SharedPtr msg);

    /**
     * @brief Executes the control logic for navigating to the goal.
     *
     * This function calculates the errors (distance and angle) and determines
     * the appropriate linear and angular velocity commands to move the robot
     * towards the goal.
     */
    void control_loop();

    /**
     * @brief Publishes velocity commands to the robot.
     *
     * Sends the computed linear and angular velocities to the robot by publishing
     * them to the /cmd_vel topic.
     *
     * @param linear Linear velocity to be sent to the robot (m/s).
     * @param angular Angular velocity to be sent to the robot (rad/s).
     */
    void publish_velocity(double linear, double angular);

    /**
     * @brief Normalizes an angle to the range [-π, π].
     *
     * Ensures that the input angle is within the valid range to avoid issues
     * caused by angle wraparound.
     *
     * @param angle Input angle in radians.
     * @return Normalized angle in radians within the range [-π, π].
     */
    double normalize_angle(double angle);

    /**
     * @brief ROS 2 publisher for velocity commands.
     *
     * Publishes geometry_msgs::msg::Twist messages to control the robot's linear
     * and angular velocities.
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    /**
     * @brief ROS 2 subscription for odometry data.
     *
     * Subscribes to nav_msgs::msg::Odometry messages to obtain the robot's current
     * position and orientation.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    /**
     * @brief ROS 2 subscription for waypoint updates.
     *
     * Subscribes to bot_waypoint_msgs::msg::BotWaypoint messages to update
     * the goal position and orientation of the robot.
     */
    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr waypoint_subscription_;

    /**
     * @brief Goal parameters for navigation.
     *
     * These parameters define the desired position and orientation for the robot,
     * as well as control gains and tolerances for reaching the goal.
     */
    double goal_x_;        /**< Desired x-coordinate of the goal. */
    double goal_y_;        /**< Desired y-coordinate of the goal. */
    double goal_theta_;    /**< Desired orientation (yaw) at the goal in radians. */
    double kp_distance_;   /**< Proportional gain for the distance controller. */
    double kp_angle_;      /**< Proportional gain for the angular controller. */
    double epsilon_;       /**< Positional tolerance for reaching the goal. */
    double epsilon_theta_; /**< Angular tolerance for achieving the desired orientation. */

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