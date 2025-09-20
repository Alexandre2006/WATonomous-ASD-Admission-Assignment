#include <chrono>
#include <memory>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"),
      costmap_core_(robot::CostmapCore(this->get_logger()))
{
  occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Read sensor values
  const auto &reading = *scan;

  float angle_min = reading.angle_min;
  float angle_increment = reading.angle_increment;
  float range_min = reading.range_min;
  float range_max = reading.range_max;

  const auto &ranges = reading.ranges;
  int count = ranges.size();

  // Compute grid parameters
  int range_size = static_cast<int>(std::ceil((2.0 * range_max) / resolution)) + 1;
  int origin = range_size / 2;

  // Initialize costmap
  costmap_.assign(range_size, std::vector<int>(range_size, 0));

  // Fill with obstacles
  for (int i = 0; i < count; i++)
  {
    float angle = angle_min + (i * angle_increment);
    float range = ranges[i];

    if (range >= range_min && range <= range_max)
    {
      int x = static_cast<int>((range * std::cos(angle)) / resolution) + origin;
      int y = static_cast<int>((range * std::sin(angle)) / resolution) + origin;

      // Check bounds
      if (x >= 0 && x < range_size && y >= 0 && y < range_size)
      {
        costmap_[y][x] = occupied;

        // Inflate surrounding cells
        int inflation_cells = static_cast<int>(inflation_radius / resolution);
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
        {
          for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
          {
            int nx = x + dx;
            int ny = y + dy;
            float dist = std::sqrt(dx * dx + dy * dy) * resolution;
            if (nx >= 0 && nx < range_size && ny >= 0 && ny < range_size && dist <= inflation_radius)
            {
              // Decrease cost linearly as distance increases
              int inflated_cost = static_cast<int>(occupied * (1.0 - dist / inflation_radius));
              costmap_[ny][nx] = std::max(costmap_[ny][nx], inflated_cost);
            }
          }
        }
      }
    }
  }

  // Publish costmap
  publishCostmap(range_size, origin, resolution);
}

void CostmapNode::publishCostmap(int width, int origin, float resolution)
{
  nav_msgs::msg::OccupancyGrid grid;
  // Add header data
  grid.header.stamp = this->now();
  grid.header.frame_id = "map";

  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = width;

  // Place robot at origin
  grid.info.origin.position.x = -static_cast<double>(origin) * resolution;
  grid.info.origin.position.y = -static_cast<double>(origin) * resolution;
  grid.info.origin.position.z = 0.0;

  // Flatten
  grid.data.resize(width * width, 0);
  for (int y = 0; y < width; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      grid.data[y * width + x] = static_cast<int8_t>(costmap_[y][x]);
    }
  }

  occ_grid_pub_->publish(grid);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}