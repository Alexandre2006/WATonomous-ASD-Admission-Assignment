#include "map_memory_node.hpp"

#include <iostream>
#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include<string>
using namespace std;

nav_msgs::msg::OccupancyGrid latest_costmap_;
geometry_msgs::msg::Quaternion orientation;

bool costmap_updated_ = false;
bool should_update_map_ = true;
double last_x = 0.0;
double last_y = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double angle = 0.0;
const double resolution = 0.1;
const float map_size = 40.0;
vector<vector<int>>global_map((int)(map_size/resolution)+1,vector<int>((int)(map_size/resolution)+1)); 

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  //Subscribers
  costmap_sub_ = this -> create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this -> create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  

  //Publishers
  map_pub_ = this -> create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  error_pub_ = this -> create_publisher<std_msgs::msg::String>("/error_topic",10);

  //Timer
  timer_ = this -> create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

//Costmap callback
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  //Store latest costmap
  latest_costmap_ = *costmap;
  costmap_updated_ = true;
}

double quatToYaw(geometry_msgs::msg::Quaternion quat)  //got this from wikipedia lol https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
{
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    double angle = atan2(siny_cosp, cosy_cosp);
    return angle;
}

//Odometry callback
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msgs)
{
  x_pos = odom_msgs -> pose.pose.position.x;
  y_pos = odom_msgs -> pose.pose.position.y;
  orientation = odom_msgs -> pose.pose.orientation;
  
  angle =  quatToYaw(orientation);

  const double distance_threshold = 5.0;

  //Compute distance traveled
  double distance = std::sqrt(std::pow(x_pos - last_x, 2) + std::pow(y_pos - last_y, 2));

  if (distance >= distance_threshold) //the robot has traveled further than the threshold so we update the map
  {
    last_x = x_pos;
    last_y = y_pos;
    should_update_map_ = true;
  }
}

//Map updater; Checks whether or not the update the global map based on the timer
void MapMemoryNode::updateMap()
{
  if (!should_update_map_) {
    RCLCPP_WARN(this->get_logger(), "Did not update path: didn't think we needed to");
    return;
  }

  if (!costmap_updated_) {
    RCLCPP_WARN(this->get_logger(), "Did not update path: no new costmap");
    return;
  }
  if(should_update_map_ && costmap_updated_)
  {
    integrateCostmap(); //update global map
    publishMap();
    should_update_map_ = false;
    costmap_updated_ = false;
  }
}

//Integrate costmap into global map
void MapMemoryNode::integrateCostmap()
{
  const int map_width = latest_costmap_.info.width;
  const int map_height = latest_costmap_.info.height;
  const int local_center_x = map_width / 2;
  const int local_center_y = map_height / 2;
  const int global_center_x = global_map.size()/2;
  const int global_center_y = global_map.size()/2;
  //convert 1D array into 2D map
  int costmap_index = 0;
  vector<vector<int>>cost_map_2D(map_height,vector<int>(map_width,0));
  for(int i = 0;i < (int)map_height;i++)
  {
    for(int j = 0;j < (int)map_width;j++)
    {
      cost_map_2D[i][j] = (int)latest_costmap_.data[costmap_index];
      costmap_index++;
    }
  }
  //rotate costmap and put points onto the global map;   
  double sina = sin(-angle);
  double cosa = cos(-angle); 
  for(int i = 0;i < (int)map_height;i++)
  {
    for(int j = 0;j < (int)map_width;j++)
    {
      if(cost_map_2D[i][j] <= 0)//costmap has no value
        continue;
      else
      {
        double relative_x = local_center_x - j;
        double relative_y = local_center_y - i;
        //get map pos using rotate by sampling, then translating the costmap to center the robot.
        //y_pos and x_pos are swapped because we prefered the map to be from the perspective of the robot when it was initialised (pointing towards positive x)
        int map_x = (int)local_center_x + (int)(relative_x * cosa - relative_y * sina) - (int)(y_pos/resolution);
        int map_y = (int)local_center_y + (int)(relative_x * sina + relative_y * cosa) - (int)(x_pos/resolution);
        //check bounds
        if(map_x >= 0 && map_x < (int)(global_map.size()) && map_y >= 0 && map_y < int(global_map.size())
           && cost_map_2D[i][j] > global_map[map_y][map_x]) //only replaces if the cost is higher
        {
          global_map[map_y][map_x] = cost_map_2D[i][j];
        }
      }
    }
  }
}

void MapMemoryNode::publishMap()
{
  nav_msgs::msg::OccupancyGrid map;
  //header
  map.header.stamp = this -> now();
  map.header.frame_id = "global_map";
  //info
  map.info.resolution = resolution;
  map.info.width = global_map.size();
  map.info.height = global_map.size();

  //robot position
  map.info.origin.position.x = x_pos;
  map.info.origin.position.y = y_pos;
  map.info.origin.position.z = 0;
  map.info.origin.orientation = orientation;

  //flatten global map
  map.data.resize(global_map.size()*global_map.size(),0);
  for(int i = 0;i < (int)global_map.size();i++)
  {
    for(int j = 0;j < (int)global_map.size();j++)
    {
      map.data[i*global_map.size()+j] = (int8_t)global_map[i][j];
    }
  }
  RCLCPP_WARN(this->get_logger(), "Publihsing!");

  map_pub_ -> publish(map); //publish map (yay!)
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
