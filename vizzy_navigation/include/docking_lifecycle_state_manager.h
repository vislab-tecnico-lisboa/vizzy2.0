/* * Copyright 2025, João Penha Lopes and João Zenário.
 * All rights reserved.
 */

/*********************************************************************************************
* Header File for the ManageLifecycleNodes C++ Class                                         *
* -                                                                                          *
* This header file provides the declaration for the ManageLifecycleNodes class,              *
* a custom Behavior Tree (BT) node designed for ROS2. This class allows a Behavior Tree      *
* to control the state transitions of a ROS2 Lifecycle Node via a service call.              *
* -                                                                                          *
*********************************************************************************************/

#ifndef MANAGE_LIFECYCLE_NODES_HPP
#define MANAGE_LIFECYCLE_NODES_HPP

// Include necessary headers.
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <thread>
#include <future>
#include "opennav_docking_bt/dock_robot.hpp"
#include "opennav_docking_bt/undock_robot.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace vizzy_navigation {

  /**
   * @brief Manages the state of ROS2 lifecycle nodes from a Behavior Tree.
   * This class inherits from `BT::AsyncActionNode` to perform a non-blocking
   * service call to a lifecycle manager.
   */
  class ManageLifecycleNodes : public BT::AsyncActionNode
  {
  public:
    /**
     * @brief Constructor for the ManageLifecycleNodes class.
     * This is a declaration; the implementation is in the .cpp file.
     * It takes the node's name and configuration from the Behavior Tree.
     * @param name The name of the node instance.
     * @param config The configuration passed to the node.
     */
    ManageLifecycleNodes(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Destructor for the ManageLifecycleNodes class.
     * Cleans up any resources, such as joining the spinning thread.
     */
    ~ManageLifecycleNodes();

    /**
     * @brief A static method that defines the input ports for this BT node.
     * This tells the BT factory what data can be provided to the node from the XML.
     * @return A list of input and output ports.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief The main execution method for the node.
     * Overridden from `BT::AsyncActionNode`, this function initiates the service call
     * and immediately returns `RUNNING`.
     * @return The status of the node, typically `RUNNING` on initiation.
     */
    BT::NodeStatus tick() override;

    /**
     * @brief The halt method for the node.
     * Overridden from `BT::AsyncActionNode`, this function is called when the
     * Behavior Tree halts the node's execution before it completes.
     */
    void halt() override;

  private:
    /**
     * @brief A shared pointer to the ROS2 node.
     * This is the handle for the ROS2 entity that will create the client.
     */
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief A shared pointer to the ROS2 service client.
     * This client is used to send requests to the lifecycle manager.
     */
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_;

    /**
     * @brief A standard C++ thread.
     * This thread is used to spin the ROS2 node in the background, allowing
     * it to receive service responses asynchronously without blocking the BT.
     */
    //std::thread executor_thread_;
  };

} // namespace vizzy_navigation

#endif // MANAGE_LIFECYCLE_NODES_HPP