/* * Copyright 2025, João Penha Lopes and João Zenário.
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

namespace vizzy_navigation {
  /**
   * @brief ManageLifecycleNodes constructor.
   * This constructor initializes the AsyncActionNode, sets up the ROS2 node and client,
   * and starts a separate thread to spin the node.
   * * @param name The name of the node from the BT XML.
   * @param config The node configuration from the BT.
   */
  ManageLifecycleNodes::ManageLifecycleNodes(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config) // Initializes the base class, passing the node name and configuration.
  {
    // The "node" object is provided by the bt_navigator.
    getInput<rclcpp::Node::SharedPtr>("node", node_);
    
    // Create a ROS2 client for the 'manage_nodes' service. 
    // This client will be used to send state change requests to the lifecycle manager.
    client_ = node_->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
        "/lifecycle_manager_docking/manage_nodes");
  }

  /**
   * @brief ManageLifecycleNodes destructor.
   */
  ManageLifecycleNodes::~ManageLifecycleNodes()
  {
  }

  /**
   * @brief providedPorts.
   * This static method is required by BehaviorTree.CPP to declare the input ports
   * that this node expects from the Behavior Tree XML.
   * * @return A list of input ports.
   */
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

  /**
   * @brief Tick method.
   * This is the core logic of the asynchronous action. It's called by the BT every tick
   * and is responsible for initiating the service call and handling its response.
   * * @return BT::NodeStatus, which is RUNNING, SUCCESS, or FAILURE.
   */
  BT::NodeStatus ManageLifecycleNodes::tick()
  {
    // Retrieve the string value from the "transition" input port.
    BT::Optional<std::string> transition_str = getInput<std::string>("transition");

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
    // We send the request and provide a lambda function as a callback.
    // The lambda's capture list [this, request] is crucial. It ensures 'this' (the class instance)
    // and 'request' (the shared pointer to the request) are available inside the lambda's body.
    client_->async_send_request(request, [this, request](const rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future) {
      // The lambda is executed when the service response is received.
      // It gets the result from the shared future object.
      auto result = future.get();
      
      // Check if the service call was successful.
      if (result->success) {
        // Log a success message.
        RCLCPP_INFO(node_->get_logger(), "Successfully executed the command.");
        // Set the node's status to SUCCESS. The Behavior Tree will read this on its next tick.
        this->setStatus(BT::NodeStatus::SUCCESS);
      } else {
        // Log an error if the service call failed.
        RCLCPP_ERROR(node_->get_logger(), "Failed to execute the command.");
        // Set the node's status to FAILURE.
        this->setStatus(BT::NodeStatus::FAILURE);
      }
    });

    // Immediately return RUNNING. This is what makes the node asynchronous; it tells
    // the Behavior Tree that the action has started but is not yet complete.
    // The BT is now free to tick other nodes while the service call is pending.
    return BT::NodeStatus::RUNNING;
  }

  /**
   * @brief halt method.
   * This method is called by the Behavior Tree if the node needs to be stopped
   * before it completes (e.g., if a parent node is halted).
   */
  void ManageLifecycleNodes::halt()
  {
    // Log an informational message to indicate that the action was halted.
    // For this simple service call, no further action is required, as the service
    // call cannot be cancelled once sent.
    RCLCPP_INFO(node_->get_logger(), "Async action was halted.");
  }
} // namespace vizzy_navigation

// Register the node with BehaviorTree.CPP as well as the DockRobot and UndockRobot actions from
// the opennav_docking_bt package.
#include "behaviortree_cpp_v3/bt_factory.h"

// Ensure this function is exported with default visibility.
extern "C"
{
  void __attribute__((visibility("default"))) BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
  {
    factory.registerNodeType<vizzy_navigation::ManageLifecycleNodes>("ManageLifecycleNodes");

    BT::NodeBuilder dock_builder =
      [&](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<opennav_docking_bt::DockRobotAction>(name, "dock_robot", config);
      };
    factory.registerBuilder<opennav_docking_bt::DockRobotAction>("DockRobot", dock_builder);

    BT::NodeBuilder undock_builder =
      [&](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<opennav_docking_bt::UndockRobotAction>(name, "undock_robot", config);
      };
    factory.registerBuilder<opennav_docking_bt::UndockRobotAction>("UndockRobot", undock_builder);
  }
}