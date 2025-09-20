#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();

private:
  const float resolution = 0.1;       // Resolution in (m)
  const float inflation_radius = 1.0; // Radius in (m)
  const int occupied = 100;

  std::vector<std::vector<int>> costmap_;
  robot::CostmapCore costmap_core_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  void publishCostmap(int width, int origin, float resolution);
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif