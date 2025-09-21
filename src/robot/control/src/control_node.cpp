
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>


#include <geometry_msgs/msg/pose_stamped.hpp>
#include "control_node.hpp"

#include <chrono>

ControlNode::ControlNode() : rclcpp::Node("pure_pursuit_controller") {
    lookahead_distance_ = 1.0;
    goal_tolerance_ = 0.1;
    linear_speed_ = 0.5;

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_path_ = msg;
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_odom_ = msg;
        });

        
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    using namespace std::chrono_literals;
    control_timer_ = this->create_wall_timer(100ms, [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        return;
    }

    // If path is empty, stop the robot
    if (current_path_->poses.empty()) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
    }

    const auto &robot_pos = robot_odom_->pose.pose.position;
    const auto &goal_pos = current_path_->poses.back().pose.position;
    if (computeDistance(robot_pos, goal_pos) <= goal_tolerance_) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
    }

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        lookahead_point = current_path_->poses.back();
    }

    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    const auto &robot_pos = robot_odom_->pose.pose.position;
    for (const auto &pose_stamped : current_path_->poses) {
        if (computeDistance(robot_pos, pose_stamped.pose.position) >= lookahead_distance_) {
            return pose_stamped;
        }
    }
    return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;

    const auto &robot_pos = robot_odom_->pose.pose.position;
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;

    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle - robot_yaw;

    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * angle_error;

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
