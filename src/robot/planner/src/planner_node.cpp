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

  // Convert world coords -> map indices
  auto worldToMap = [&](double wx, double wy, int &mx, int &my) -> bool
  {
    double res = current_map_.info.resolution;
    int width = current_map_.info.width;
    int height = current_map_.info.height;

    // Center of the map in map indices
    double center_x = width / 2.0;
    double center_y = height / 2.0;

    mx = static_cast<int>(std::round(center_x + wx / res));
    my = static_cast<int>(std::round(center_y + wy / res));

    return (mx >= 0 && mx < width && my >= 0 && my < height);
  };

  RCLCPP_INFO(this->get_logger(),
              "Planning path: Start (world): (%.2f, %.2f), Goal (world): (%.2f, %.2f)",
              robot_pose_.position.x, robot_pose_.position.y,
              goal_.point.x, goal_.point.y);

  int start_x, start_y, goal_x, goal_y;
  if (!worldToMap(robot_pose_.position.x, robot_pose_.position.y, start_x, start_y) ||
      !worldToMap(goal_.point.x, goal_.point.y, goal_x, goal_y))

    RCLCPP_INFO(this->get_logger(),
                "Map origin: (%.2f, %.2f), resolution: %.3f, size: %d x %d",
                current_map_.info.origin.position.x, current_map_.info.origin.position.y,
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

  auto isFree = [&](int x, int y)
  {
    int idx = y * current_map_.info.width + x;
    return current_map_.data[idx] == 0; // 0 = free, >0 = occupied
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

  auto makeKey = [&](int x, int y)
  { return y * current_map_.info.width + x; };

  ANode *start = new ANode{start_x, start_y, 0.0, heuristic(start_x, start_y), nullptr};
  open_set.push(start);
  all_nodes[makeKey(start_x, start_y)] = start;

  std::unordered_map<int, double> g_score;
  g_score[makeKey(start_x, start_y)] = 0.0;

  ANode *goal_node = nullptr;

  std::vector<std::pair<int, int>> dirs = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  while (!open_set.empty())
  {
    ANode *current = open_set.top();
    open_set.pop();

    if (current->x == goal_x && current->y == goal_y)
    {
      goal_node = current;
      break;
    }

    for (auto [dx, dy] : dirs)
    {
      int nx = current->x + dx;
      int ny = current->y + dy;

      if (!inBounds(nx, ny) || !isFree(nx, ny))
        continue;

      double tentative_g = current->g + std::hypot(dx, dy);
      int key = makeKey(nx, ny);

      if (!g_score.count(key) || tentative_g < g_score[key])
      {
        ANode *neighbor = new ANode{nx, ny, tentative_g, heuristic(nx, ny), current};
        open_set.push(neighbor);
        g_score[key] = tentative_g;
        all_nodes[key] = neighbor;
      }
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  if (goal_node)
  {
    // Reconstruct path
    ANode *cur = goal_node;
    while (cur)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;

      // Center-based origin conversion
      double res = current_map_.info.resolution;
      int width = current_map_.info.width;
      int height = current_map_.info.height;
      double center_x = width / 2.0;
      double center_y = height / 2.0;

      pose.pose.position.x = (cur->x - center_x) * res;
      pose.pose.position.y = (cur->y - center_y) * res;
      pose.pose.position.z = 0.0;
      path.poses.push_back(pose);
      cur = cur->parent;
    }
    std::reverse(path.poses.begin(), path.poses.end());
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
