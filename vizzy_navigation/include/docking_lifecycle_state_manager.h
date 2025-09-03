/*
 * Copyright 2025, João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
* Header File for the ManageLifecycleNodes C++ Class (BehaviorTree.CPP v4)                   *
* -                                                                                          *
* This header file provides the declaration for the ManageLifecycleNodes class,              *
* a custom Behavior Tree (BT) node designed for ROS2. This class allows a Behavior Tree      *
* to control the state transitions of a ROS2 Lifecycle Node via a service call.              *
* -                                                                                          *
*********************************************************************************************/

#ifndef VIZZY_NAVIGATION__MANAGE_LIFECYCLE_NODES_HPP_
#define VIZZY_NAVIGATION__MANAGE_LIFECYCLE_NODES_HPP_

#include <string>
#include <memory>
#include <thread>
#include <future>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace vizzy_navigation
{

/**
 * @brief Manages the state of ROS2 lifecycle nodes from a Behavior Tree.
 * This class inherits from 'BT::StatefulActionNode' to perform a non-blocking
 * service call to a lifecycle manager.
 */
class ManageLifecycleNodes : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor for the ManageLifecycleNodes class.
   * @param name The name of the node instance.
   * @param config The configuration passed to the node.
   */
  ManageLifecycleNodes(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief A static method that defines the input ports for this BT node.
   * @return A list of input ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Method called when the node is ticked for the first time.
   * It is responsible for initializing the service call.
   * @return BT::NodeStatus::RUNNING if the action was successfully started.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Method called periodically while the node is in the RUNNING state.
   * It checks the result of the asynchronous service call.
   * @return BT::NodeStatus::SUCCESS or BT::NodeStatus::FAILURE when the action is completed,
   * or BT::NodeStatus::RUNNING if it is still in progress.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Method called when the node is halted by the Behavior Tree.
   * It is responsible for any cleanup, like canceling the service call.
   */
  void onHalted() override;

private:
  // A shared pointer to the ROS2 node.
  rclcpp::Node::SharedPtr node_;

  // A shared pointer to the ROS2 service client.
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_;
  
  // Future to store the result of the async service call.
  std::shared_future<rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedResponse> future_result_;
};

}  // namespace vizzy_navigation

#endif  // VIZZY_NAVIGATION__MANAGE_LIFECYCLE_NODES_HPP_