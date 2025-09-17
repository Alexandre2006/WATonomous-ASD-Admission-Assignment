#include "map_memory_node.hpp"
#include <cmath>
#include<vector>
class MappingNode : public rclcpp::Node
{
  public:
    MappingNode() : Node("mapping_node"), last_x(0.0), last_y(0,0),travel_distance(5.0)
    {
      //Subscribers
      costmap_sub_ = this -> create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
      odom_sub_ = this -> create_subscription<nav_msgs::msg:Odometry>("/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1))


      //Publishers
      map_pub_ = this -> create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);


      //Timer
      timer_ = this -> create_wall_timer(std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this))
    }
 
  private:
    //Subscribers
    rclcpp:Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    //Publishers
    rclcpp:Publisher<nav_msgs::msg::OcuupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;


    //Global map and position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_, last_y;
    const double distance_threshold = 5.0;
    const double resolution = 0.1;
    bool costmap_updated_ = false;


    //Costmap callback
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
      //Store latest costmap
      latest_costmap_ = *msg;
      costmap_updated_ = true;
    }


    //Odometry callback
    void odomCallback(const nav_msgs:;msg::Odometry::SharedPtr msg)
    {
      double x = msg -> pose.pose.position.x;
      double y = msg -> pose.pose.position.y;


      //Compute distance traveled
      double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
      if (distance >= distance_threshold) //the robot has traveled further than the threshold so we update the map
      {
        last_x = x;
        last_y = y;
        should_update_map_ = true;
      }
    }


    //Map updater; Checks whether or not the update the global map based on the timer
    void updateMap()
    {
      if(should_update_map_ && costmap_updated_)
      {
        integrateCostmap(); //update global map
        map_pub_ -> publish(global_map_);
        should_update_map_ = false;
      }
    }


    //Integrate costmap into global map
    void integrateCostmap()
    {
      double x_offset = msg -> pose.pose.position.x;
      double y_offset = msg -> pose.pose.position.y;
      double orientation = msg -> pose.pose.orientation;
      //convert 1D array into 2D map
      vector<int>cost_map = latest_costmap_.data;
     
      // do magic with latest_costmap_ to convert to 2D
      vector<vector<int>>2D_cost_map;


      //convert costmap points into global map points      
      for(int i = 0;i < 2D_cost_map.size();i++)
      {
        for(int j = 0;j < 2D_cost_map[i].size();i++)
        {
          if(2D_cost_map[i] == 0) //nothing was recorded to be in that spot, so we dont try to replace any value
            continue;
          else
          {
            double point_distance = std::sqrt(std::pow(2D_cost_map.size() - i, 2) + std::pow(j - 2D_cost_map[i].size()/2, 2)); //get distance from robot
            int map_x = (int)(point_distance * cos(orientation) + x_offset);//get x pos on the map
            int map_y = (int)(point_distance * sin(orientation) + y_offset);//get y pos on the map
            global_map_[2D_cost_map[i].size() * i + j] = 2D_cost_map[i][j];//plot value onto global map
          }
        }
      }      
    }


    //Flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = false;
};


MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
 
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
