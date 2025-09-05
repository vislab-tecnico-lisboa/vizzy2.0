#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/node_thread.hpp"

namespace vizzy_navigation
{

class CheckPathCost : public BT::ConditionNode
{
public:
  CheckPathCost(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  CheckPathCost() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to check"),
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", "TF buffer"),
      BT::InputPort<std::string>("global_frame", "Global frame ID, usually 'map'"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame ID"),
      BT::InputPort<double>("check_distance", 3.0, "Distance in meters to check ahead on the path"),
      BT::InputPort<int>("cost_threshold", 128, "Costmap values above this are considered high"),
      BT::InputPort<double>("ratio_threshold", 0.25, "If ratio of high-cost points exceeds this, return SUCCESS")
    };
  }

private:
  void initialize();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string global_frame_;
  std::string robot_base_frame_;
  bool initialized_ = false;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_; 
};

}