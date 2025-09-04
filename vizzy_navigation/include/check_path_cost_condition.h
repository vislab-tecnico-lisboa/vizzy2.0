/*
 * Copyright 2025, João Penha Lopes and João Zenário.
 * All rights reserved.
 */

#ifndef CHECK_PATH_COST_CONDITION_H_
#define CHECK_PATH_COST_CONDITION_H_

#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/node_thread.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

/**
  * @brief Condition node that checks the cost of a given path.
  * This node is used to determine what controller to use given specific path conditions,
  * e.g. narrow vs wide controllers.
  */
class CheckPathCost : public BT::ConditionNode
{
public:

/**
 * @brief Constructor for CheckPathCost condition node.
 * This is a declaration; the implementation is in the .cpp file.
 * It takes the node configuration.
 * @param conf Node configuration.
 */
  CheckPathCost(const std::string& name, const BT::NodeConfiguration& conf);
  
  /**
   * @brief The main execution code for the node.
   * This function is called every time the node is ticked.
   * @return BT::NodeStatus The status of the node after ticking.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Provides the list of input ports for the node.
   * @return BT::PortsList The list of input ports.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("check_distance", 3.0, "Distance in meters to check ahead on the path"),
      BT::InputPort<int>("cost_threshold", 128, "Costmap values above this are considered high"),
      BT::InputPort<double>("ratio_threshold", 0.25, "If ratio of high-cost points exceeds this, return SUCCESS")
    };
  }

private:
  /**
   * @brief A shared pointer to the ROS2 node.
   * Because this node is executed within the BT, the node_ instance
   * will be a shared pointer to the bt_navigator node, which summons the tree.
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief A shared pointer to the costmap ROS2 node.
   * This costmap_ros_ instance will be a shared pointer to the global costmap node used
   * by the navigation stack.
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /**
   * @brief A raw pointer to the costmap.
   * This costmap_ instance will be a pointer to the global costmap itself used
   * by the navigation stack within the node pointed by the costmap_ros_ instance.
   */
  nav2_costmap_2d::Costmap2D* costmap_;

  /**
   * @brief A shared pointer to the TF2 buffer.
   * This tf_buffer_ instance will be a shared pointer to the TF2 buffer used
   * by ROS2 for transforms.
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief The global frame ID, usually 'map'.
   * This string is used as the reference frame for all global coordinates.
   */
  std::string global_frame_;

  /**
   * @brief The robot base frame ID, usually 'base_footprint'.
   * This string is used as the reference frame for the robot's base.
   */
  std::string robot_base_frame_;

  /**
   * @brief Boolean to indicate whether the node has been correctly initialized.
   */
  bool initialized_ = false;
};

#endif // CHECK_PATH_COST_CONDITION_H_