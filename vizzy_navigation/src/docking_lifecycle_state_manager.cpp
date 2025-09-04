 /**
  * Copyright 2025, João Penha Lopes and João Zenário.
  * All rights reserved.
  */

/*********************************************************************************************
* Manage Lifecycle Nodes BT Action                                                           *
* -                                                                                          *
* This file implements a custom Behavior Tree (BT) Action node for ROS2.                     *
* Its purpose is to programmatically change the state of a Lifecycle Manager's nodes         *
* via a ROS2 service call. This enables a Behavior Tree to dynamically control the lifecycle *
* of a node, such as activating a sensor for a specific task and deactivating it afterwards  *
* to conserve resources.                                                                     *
* -                                                                                          *
* The core functionality is to send a request to the'/lifecycle_manager_docking/manage_nodes'*
* service with a specific command (e.g., 'activate' or 'deactivate').                        *
*********************************************************************************************/

#include "docking_lifecycle_state_manager.h" 


ManageLifecycleNodes::ManageLifecycleNodes(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) 
{
}

BT::PortsList ManageLifecycleNodes::providedPorts()
{
  // Ports in the context of Behaviour Trees are like variables, to which there are
  // input variables (input ports) and output variables (output ports).
  // Input ports are used to receive data from the BT.
  // Output ports are used to send data back to the BT.
  // In this case, there are no output ports defined.
  return {
    // ? Is it possible to manage the state of a single node within a lifecycle manager ?
    // Declares an input port named "transition" of type std::string.
    // This is used to specify the state transition command ("activate" or "deactivate").
    BT::InputPort<std::string>("transition"),

    // Declares an input port named "node" of type rclcpp::Node::SharedPtr.
    // This is used to specify the ROS2 node that will manage the client.
    BT::InputPort<rclcpp::Node::SharedPtr>("node")
  };
}

BT::NodeStatus ManageLifecycleNodes::onStart()
{
  // Initialize the node shared pointer instance.
  if (!getInput<rclcpp::Node::SharedPtr>("node", node_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ManageLifecycleNodes"), "Missing required input [node]");
    return BT::NodeStatus::FAILURE;
  }

  // Create a ROS2 client for the 'manage_nodes' service.
  // This client will be used to send state change requests to the lifecycle manager.
  if (!client_) {
    client_ = node_->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "/lifecycle_manager_docking/manage_nodes");
  }

  // Retrieve the string value from the "transition" input port.
  BT::Expected<std::string> transition_str = getInput<std::string>("transition");

  // Check if either of the required input ports is missing a value.
  if (!transition_str.has_value()) {
    // Log an error and return FAILURE. The node cannot proceed without these inputs.
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [transition]");
    return BT::NodeStatus::FAILURE;
  }

  // Define a variable to hold the command code for the service request.
  uint8_t command;
  // Map the string "activate" to the numerical command 1.
  if (transition_str.value() == "activate") {
      command = 1;
  // Map the string "deactivate" to the numerical command 2.
  } else if (transition_str.value() == "deactivate") {
      command = 2;
  } else {
    // If an invalid transition string is provided, log an error and fail.
    RCLCPP_ERROR(node_->get_logger(), "Invalid transition string: %s", transition_str.value().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Block and wait for the service to be available. If it's not ready within 1 second, fail.
  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Service /lifecycle_manager_docking/manage_nodes not available");
    return BT::NodeStatus::FAILURE;
  }

  // Create a shared pointer for the service request message.
  auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  // Set the command field of the request message.
  request->command = command;

  // This is the implementation of the asynchronous call.
  future_result_ = client_->async_send_request(request).share();

  // Immediately return RUNNING. This is what makes the node asynchronous; it tells
  // the Behavior Tree that the action has started but is not yet complete.
  // The BT is now free to tick other nodes while the service call is pending.
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ManageLifecycleNodes::onRunning()
{
  // Check if the future is ready.
  if (future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto result = future_result_.get();
    if (result->success) {
      RCLCPP_INFO(node_->get_logger(), "Successfully executed the command.");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to execute the command.");
      return BT::NodeStatus::FAILURE;
    }
  }

  // The service call is still in progress.
  return BT::NodeStatus::RUNNING;
}

void ManageLifecycleNodes::onHalted()
{
  // Log an informational message to indicate that the action was halted.
  // For this simple service call, no further action is required, as the service
  // call cannot be cancelled once sent.
  RCLCPP_INFO(node_->get_logger(), "Async action was halted.");
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ManageLifecycleNodes>("ManageLifecycleNodes");
}