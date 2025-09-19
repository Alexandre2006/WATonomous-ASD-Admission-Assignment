#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;
    //subscribers
    rclcpp::Subscribers<nav_msgs::msgs::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscribers<nav_msgs::msgs::Odometry>::SharedPtr costmap_sub_;

    //publishers
    rclcpp::Publisher<nav_msgs::msgs::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<std_msgs::msgs::String>::SharedPtr error_pub_;
    
    //timer
    rclcpp::TimerBase::SharedPtr::timer_;
};

#endif 
