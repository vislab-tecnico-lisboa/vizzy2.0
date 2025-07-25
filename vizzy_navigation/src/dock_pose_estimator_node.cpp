/* 
 * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
*                      Dock Pose Estimator Node C++ File for ROS2                            *
*                                          -                                                 *
* This file contains the implementation of the main Dock Pose Estimator Node for ROS2.       *
* It initializes the ROS2 client library, creates a DockPoseEstimator object,                *
* and spins the node.                                                                        *
* The methods from DockPoseEstimator defined in dock_pose_estimator.h are implemented in the *
* dock_pose_estimator.cpp file.                                                              *
*                                          -                                                 *
* This docking estimation procedure was redesigned to work with Nav2's Docking Server.       *
* More details can be found in the documentation:                                            *   
* https://github.com/open-navigation/opennav_docking/tree/humble                             *    
*********************************************************************************************/

// Include necessary headers for ROS2, and other dependencies.
#include "dock_pose_estimator.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

// This is the main function of the ROS 2 node.
int main(int argc, char **argv)
{
  // Initialize the ROS 2 C++ client library.
  rclcpp::init(argc, argv);

  // Create a shared pointer to a new DockPoseEstimator object.
  auto dock_pose_estimator_node = std::make_shared<DockPoseEstimator>(rclcpp::NodeOptions());

  // For now, let us enable immediatly the dock pose estimator.
  dock_pose_estimator_node->enable();

  // Spin the node to allow it to process incoming messages and events from 
  // subscribed topics and services, and direct the callbacks to the appropriate methods.
  // This is a blocking call that will keep the node running until it is shut down.
  rclcpp::spin(dock_pose_estimator_node);

  // Cleanly shut down the ROS 2 client library.
  rclcpp::shutdown();

  // Return 0 to indicate successful execution of the program.
  return 0;
}