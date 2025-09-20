#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

// Required includes you wanted
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>

// Extras needed for declarations
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

// If you have ControlCore, include it here
// #include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    // If you have ControlCore, uncomment this
    // robot::ControlCore control_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Data storage
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;

    // Internal methods — match exactly with .cpp
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
};

#endif // CONTROL_NODE_HPP_
