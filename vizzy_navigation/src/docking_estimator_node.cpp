/* 
 * Copyright 2025, Joao Avelino, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino for ROS1 in 2019.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
 *                      Docking Estimator Node C++ File for ROS2                             *
 *                                          -                                                *
 * This file contains the implementation of the main Docking Estimator Node for ROS2.        *
 * It initializes the ROS2 client library, creates a DockingEstimator object,                *
 * and spins the node.                                                                       *
 * The methods from DockingEstimator defined in docking_estimator.h are implemented in the   *
 * docking_estimator.cpp file.                                                               *
*********************************************************************************************/

// Include necessary headers for ROS2, and other dependencies.
#include "docking_estimator.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

// This is the main function of the ROS 2 node.
int main(int argc, char **argv)
{
  // Initialize the ROS 2 C++ client library.
  rclcpp::init(argc, argv);

  // Create a shared pointer to a new DockingEstimator object.
  auto docking_estimator_node = std::make_shared<DockingEstimator>(rclcpp::NodeOptions());

  // Spin the node to allow it to process incoming messages and events from 
  // subscribed topics and services, and direct the callbacks to the appropriate methods.
  // This is a blocking call that will keep the node running until it is shut down.
  rclcpp::spin(docking_estimator_node);

  // Cleanly shut down the ROS 2 client library.
  rclcpp::shutdown();

  // Return 0 to indicate successful execution of the program.
  return 0;
}