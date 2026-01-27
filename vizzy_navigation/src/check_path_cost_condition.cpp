#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "check_path_cost_condition.h"

namespace vizzy_navigation
{

CheckPathCost::CheckPathCost(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

void CheckPathCost::initialize()
{
    // The node is expected to be available on the blackboard.
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
        RCLCPP_FATAL(rclcpp::get_logger("CheckPathCost"), "Failed to get rclcpp::Node::SharedPtr from blackboard. This is critical!");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "CheckPathCost node: Initializing...");

    // List all keys on the blackboard. This is useful for debugging and understanding what data is available.
    RCLCPP_INFO(node_->get_logger(), "CheckPathCost node: Blackboard contains the following keys:");
    for (const auto& key : config().blackboard->getKeys()) {
        RCLCPP_INFO(node_->get_logger(), "- %s", key.data());
    }

    // Get the TF buffer from the blackboard.
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (!tf_buffer_) {
        RCLCPP_FATAL(node_->get_logger(), "Failed to get tf2_ros::Buffer from blackboard. This is critical for TF lookups!");
        return;
    }

    // Initilize the costmap ROS node.
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    
    // The costmap needs its own thread to spin and process ROS messages (like topic updates).
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    // The costmap is a lifecycle node, so we must also bring it up.
    costmap_ros_->on_configure(costmap_ros_->get_current_state());
    costmap_ros_->on_activate(costmap_ros_->get_current_state());

    // Get the costmap pointer from the Costmap2DROS instance.
    costmap_ = costmap_ros_->getCostmap();

    getInput("global_frame", global_frame_);
    getInput("robot_base_frame", robot_base_frame_);
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "CheckPathCost node initialized successfully.");
}

BT::NodeStatus CheckPathCost::tick()
{
  if (!initialized_) {
    initialize();
  }

  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Input path not available or empty.");
    return BT::NodeStatus::FAILURE;
  }

  // Get config values from ports.
  double check_distance, ratio_threshold;
  int cost_threshold;
  getInput("check_distance", check_distance);
  getInput("cost_threshold", cost_threshold);
  getInput("ratio_threshold", ratio_threshold);

  // Get current robot pose.
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_buffer_, global_frame_, robot_base_frame_, 1.0))
  {
    RCLCPP_WARN(node_->get_logger(), "Could not get current robot pose.");
    return BT::NodeStatus::FAILURE;
  }
  
  // Find the closest point on the path to start checking from.
  auto closest_iter = nav2_util::geometry_utils::min_by(
    path.poses.begin(), path.poses.end(),
    [&current_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return nav2_util::geometry_utils::euclidean_distance(ps, current_pose);
    });

  unsigned int high_cost_points = 0;
  unsigned int points_checked = 0;
  double distance_traveled = 0.0;

  // Loop through path from closest point up to check_distance.
  for (auto it = closest_iter; it != path.poses.end() && distance_traveled < check_distance; ++it) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(it->pose.position.x, it->pose.position.y, mx, my)) {
      continue; // Point is outside the costmap bounds.
    }

    if (costmap_->getCost(mx, my) >= cost_threshold) {
      high_cost_points++;
    }
    points_checked++;

    if (std::next(it) != path.poses.end()) {
      distance_traveled += nav2_util::geometry_utils::euclidean_distance(*it, *std::next(it));
    }
  }

  if (points_checked == 0) {
    return BT::NodeStatus::FAILURE; // No valid points to check.
  }

  double high_cost_ratio = static_cast<double>(high_cost_points) / points_checked;

  RCLCPP_DEBUG(
    node_->get_logger(), "Path cost check: ratio %.2f, threshold %.2f",
    high_cost_ratio, ratio_threshold);

  if (high_cost_ratio > ratio_threshold) {
    RCLCPP_INFO(node_->get_logger(), "High cost path segment detected! Using NARROW controller.");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(), "Path segment is clear. Using WIDE controller.");
  return BT::NodeStatus::FAILURE;
}

} 

// Register the node with BehaviorTree.CPP.
#include "behaviortree_cpp_v3/bt_factory.h"

// Ensure this function is exported with default visibility.
extern "C"
{
  void __attribute__((visibility("default"))) BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
  {
    factory.registerNodeType<vizzy_navigation::CheckPathCost>("CheckPathCost");
  }
}