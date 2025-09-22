#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::planPath()
{
  if (!goal_received_)
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing goal!");
    return;
  }

  if (current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map!");
    return;
  }

  // Simplified coordinate conversion - both coordinates are relative to map center
    auto worldToMap = [&](double wx, double wy, int &mx, int &my) -> bool
  {
    double res = current_map_.info.resolution;
    int width = current_map_.info.width;
    int height = current_map_.info.height;

    // Convert from center-relative coordinates to map indices
    // Map center is at (width/2, height/2) in grid coordinates
    mx = static_cast<int>(std::floor(wx / res + width / 2.0));
    my = static_cast<int>(std::floor(-wy / res + height / 2.0)); // Flip Y-axis

    return (mx >= 0 && mx < width && my >= 0 && my < height);
  };

  // Helper function to convert map coordinates back to world coordinates
  auto mapToWorld = [&](int mx, int my, double &wx, double &wy) -> void
  {
    double res = current_map_.info.resolution;
    int width = current_map_.info.width;
    int height = current_map_.info.height;

    // Convert from map indices back to center-relative coordinates
    wx = (mx - width / 2.0 + 0.5) * res;  // +0.5 for cell center
    wy = -(my - height / 2.0 + 0.5) * res; // Flip Y-axis back
  };

  RCLCPP_INFO(this->get_logger(),
              "Planning path: Start (world): (%.2f, %.2f), Goal (world): (%.2f, %.2f)",
              robot_pose_.position.x, robot_pose_.position.y,
              goal_.point.x, goal_.point.y);

  int start_x, start_y, goal_x, goal_y;

  RCLCPP_INFO(this->get_logger(),
              "Map resolution: %.3f, size: %d x %d",
              current_map_.info.resolution,
              current_map_.info.width, current_map_.info.height);

  bool start_ok = worldToMap(robot_pose_.position.x, robot_pose_.position.y, start_x, start_y);
  bool goal_ok = worldToMap(goal_.point.x, goal_.point.y, goal_x, goal_y);

  RCLCPP_INFO(this->get_logger(),
              "Start (world): (%.2f, %.2f) -> (map): (%d, %d) [%s]",
              robot_pose_.position.x, robot_pose_.position.y, start_x, start_y, start_ok ? "OK" : "OUT");
  RCLCPP_INFO(this->get_logger(),
              "Goal  (world): (%.2f, %.2f) -> (map): (%d, %d) [%s]",
              goal_.point.x, goal_.point.y, goal_x, goal_y, goal_ok ? "OK" : "OUT");

  if (!start_ok || !goal_ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Start or goal out of map bounds!");
    return;
  }

  auto inBounds = [&](int x, int y)
  {
    return x >= 0 && x < (int)current_map_.info.width &&
           y >= 0 && y < (int)current_map_.info.height;
  };

  // Modified cost function instead of binary free/occupied check
  auto getCost = [&](int x, int y) -> double
  {
    int idx = y * current_map_.info.width + x;
    int8_t cell_value = current_map_.data[idx];
    
    if (cell_value == -1) {
      // Unknown space - treat as moderately costly but passable
      return 5.0;
    } else if (cell_value == 0) {
      // Free space - lowest cost
      return 1.0;
    } else if (cell_value < 50) {
      // Low occupancy (inflation) - higher cost but still passable
      return 2.0 + (cell_value / 50.0) * 8.0; // Cost ranges from 2-10
    } else if (cell_value < 90) {
      // Medium occupancy - very high cost but emergency passable
      return 10.0 + (cell_value - 50) / 40.0 * 40.0; // Cost ranges from 10-50
    } else {
      // High occupancy - extremely high cost, avoid if possible
      return 100.0; // Very high but not infinite
    }
  };

  auto heuristic = [&](int x, int y)
  {
    return std::hypot(goal_x - x, goal_y - y);
  };

  // Priority queue for open set
  auto cmp = [](ANode *a, ANode *b)
  { return a->f() > b->f(); };
  std::priority_queue<ANode *, std::vector<ANode *>, decltype(cmp)> open_set(cmp);

  std::unordered_map<int, ANode *> all_nodes;
  std::unordered_set<int> closed_set;

  auto makeKey = [&](int x, int y)
  { return y * current_map_.info.width + x; };

  ANode *start_node = new ANode{start_x, start_y, 0.0, heuristic(start_x, start_y), nullptr};
  open_set.push(start_node);
  all_nodes[makeKey(start_x, start_y)] = start_node;

  ANode *goal_node = nullptr;

  std::vector<std::pair<int, int>> dirs = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  while (!open_set.empty())
  {
    ANode *current = open_set.top();
    open_set.pop();

    int current_key = makeKey(current->x, current->y);
    if (closed_set.count(current_key)) {
      continue;
    }
    closed_set.insert(current_key);

    if (current->x == goal_x && current->y == goal_y)
    {
      goal_node = current;
      break;
    }

    for (auto [dx, dy] : dirs)
    {
      int nx = current->x + dx;
      int ny = current->y + dy;

      if (!inBounds(nx, ny))
        continue;

      int neighbor_key = makeKey(nx, ny);
      if (closed_set.count(neighbor_key))
        continue;

      // Calculate cost including terrain cost
      double movement_cost = std::hypot(dx, dy);
      double terrain_cost = getCost(nx, ny);
      double tentative_g = current->g + movement_cost * terrain_cost;

      if (all_nodes.count(neighbor_key)) {
        if (tentative_g >= all_nodes[neighbor_key]->g)
          continue;
      }

      ANode *neighbor = new ANode{nx, ny, tentative_g, heuristic(nx, ny), current};
      open_set.push(neighbor);
      all_nodes[neighbor_key] = neighbor;
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  if (goal_node)
  {
    // Reconstruct path using the proper coordinate conversion
    ANode *cur = goal_node;
    while (cur)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;

      // Use the mapToWorld function for proper conversion
      mapToWorld(cur->x, cur->y, pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;
      
      pose.pose.orientation.w = 1.0;
      
      path.poses.push_back(pose);
      cur = cur->parent;
    }
    std::reverse(path.poses.begin(), path.poses.end());
    RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", path.poses.size());
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Failed to find path with A*.");
  }

  path_pub_->publish(path);

  // Cleanup
  for (auto &kv : all_nodes)
    delete kv.second;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
