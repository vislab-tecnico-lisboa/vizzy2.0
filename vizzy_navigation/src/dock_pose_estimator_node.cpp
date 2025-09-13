/* * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
* Dock Pose Estimator Node C++ File for ROS2                                                 *
* -                                                                                          *
* This file contains the main entry point for the Dock Pose Estimator Lifecycle Node.        *
* It initializes the ROS2 client library, creates a DockPoseEstimator object,                *
* adds it to an executor, and spins the executor to process events.                          *
* -                                                                                          *
* As a lifecycle node, its operation (e.g., activating, deactivating) is managed by an       *
* external entity like the Nav2 Lifecycle Manager, not within this main function.            *
*********************************************************************************************/

// Include necessary headers for ROS2, and other dependencies.
#include "dock_pose_estimator.h"

// This is the main function of the ROS 2 node.
int main(int argc, char **argv)
{
  // Initialize the ROS 2 C++ client library.
  rclcpp::init(argc, argv);

  // Create a shared pointer to a new DockPoseEstimator lifecycle node object.
  auto dock_pose_estimator_node = std::make_shared<DockPoseEstimator>(rclcpp::NodeOptions());

  // Create an executor to manage the lifecycle node.
  // A MultiThreadedExecutor is a robust choice that can handle multiple callbacks.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 4 threads.
  executor.add_node(dock_pose_estimator_node->get_node_base_interface());

  // Spin the executor to process callbacks and state transitions for the node.
  // This is a blocking call that will keep the node running until it is shut down.
  executor.spin();

  // Cleanly shut down the ROS 2 client library.
  rclcpp::shutdown();

  // Return 0 to indicate successful execution of the program.
  return 0;
}