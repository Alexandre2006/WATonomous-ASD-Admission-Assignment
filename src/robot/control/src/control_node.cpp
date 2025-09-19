#include "control_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

ControlNode::ControlNode()
: Node("control"),
  control_(robot::ControlCore(this->get_logger())),
  lookahead_distance_(1.0),
  goal_tolerance_(0.1),
  linear_speed_(0.5)
{
    // Subscriber: Path
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_path_ = msg;
        });

    // Subscriber: Odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_odom_ = msg;
        });

    // Publisher: Velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer: Control loop at 10 Hz
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        return; // Wait until we have both path and odometry
    }

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        return; // No valid target
    }

    // Option 1: Use internal Pure Pursuit math here
    auto cmd_vel = computeVelocity(*lookahead_point);

    // Option 2: If ControlCore handles velocity computation:
    // auto cmd_vel = control_.computeVelocity(*lookahead_point, *robot_odom_);

    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    const auto &robot_pos = robot_odom_->pose.pose.position;

    for (const auto &pose_stamped : current_path_->poses) {
        double dist = computeDistance(robot_pos, pose_stamped.pose.position);
        if (dist >= lookahead_distance_) {
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

    // Normalize to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * angle_error; // proportional steering

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
