/* 
 * Copyright 2025, João Avelino, Rui Figueiredo, Plinio Moreno, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino, Rui Figueiredo and Plinio Moreno for ROS1 in 2022.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

#include "charging_action_server.h"

void ChargingActionServer::goalCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<vizzy_msgs::action::Charge::Result>();

    //! --- Goal 1: Handling the 'CHARGE' Goal (Updated Logic) ---

    if (goal->goal == vizzy_msgs::action::Charge::Goal::CHARGE)
    {
        RCLCPP_INFO(this->get_logger(), "Received new goal: Charge! Checking battery state...");

        // Check if the robot is already charging.
        auto request = std::make_shared<vizzy_msgs::srv::BatteryChargingState::Request>();
        charging_state_client_->async_send_request(request,
            [this, goal_handle, result](rclcpp::Client<vizzy_msgs::srv::BatteryChargingState>::SharedFuture response)
        {
            if (response.get()->battery_charging_state)
            {
                RCLCPP_INFO(this->get_logger(), "SUCCESS! Robot is already at the dock and charging.");
                result->result = result->CHARGE_SUCCESS;
                goal_handle->succeed(result);
                return;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Robot is not charging. Delegating to Docking Mission BT...");

                // Define the staging pose for the mission.
                // The BT's NavigateToPose action will use this as its 'goal'.
                geometry_msgs::msg::PoseStamped staging_pose;
                staging_pose.header.frame_id = "map";
                staging_pose.header.stamp = this->get_clock()->now();
                // TODO: For now we just assume the staging pose is at (-1, 0) in the map frame, 
                // TODO: and the dock is at (0,0). This should be made a parameter.
                staging_pose.pose.position.x = -1.0;
                staging_pose.pose.orientation.w = 1.0;

                // Create a NavigateToPose goal.
                auto nav_goal = nav2_msgs::action::NavigateToPose::Goal();
                nav_goal.pose = staging_pose;

                // Get the absolute path to the package's installed share directory.
                std::string pkg_share_path = ament_index_cpp::get_package_share_directory("vizzy_navigation");

                // Construct the full, absolute path to the BT file.
                if (activate_dock_pose_detection_)
                {   
                    /*
                    RCLCPP_INFO(this->get_logger(), "Activating dock pose detection immediately as per parameter. Robot WILL NOT navigate.");
                    std::string bt_xml_path = pkg_share_path + "/behavior_trees/custom_docking_bt_activator_nav2.xml";
                    nav_goal.behavior_tree = bt_xml_path;
                    RCLCPP_INFO(this->get_logger(), "Using BT file: %s", bt_xml_path.c_str());*/
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Dock pose detection will NOT be activated immediately. Robot WILL navigate");
                    std::string bt_xml_path = pkg_share_path + "/behavior_trees/custom_docking_bt_navigator_nav2.xml";
                    nav_goal.behavior_tree = bt_xml_path;
                    RCLCPP_INFO(this->get_logger(), "Using BT file: %s", bt_xml_path.c_str());
                }

                // Send the goal and handle the result of the entire mission.

                // * Because we defined the BT in the action goal, when we call NavigateToPose, the bt navigator
                // * will override the default behavior tree with this one, enabling the custom docking behavior.

                auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
                send_goal_options.result_callback =
                    [this, goal_handle, result](const auto & nav_result)
                {
                    // This callback runs after the ENTIRE docking BT is finished.
                    if (nav_result.code != rclcpp_action::ResultCode::SUCCEEDED)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Docking Mission failed!");
                        result->result = result->CHARGE_FAILED;
                        goal_handle->abort(result);
                        return;
                    }

                    // The BT succeeded, now we do the final verification.
                    RCLCPP_INFO(this->get_logger(), "Docking Mission completed. Verifying charging status...");
                    auto final_request = std::make_shared<vizzy_msgs::srv::BatteryChargingState::Request>();
                    charging_state_client_->async_send_request(final_request,
                        [this, goal_handle, result](rclcpp::Client<vizzy_msgs::srv::BatteryChargingState>::SharedFuture final_response)
                        {
                            if (final_response.get()->battery_charging_state || is_simulation_)
                            {
                                RCLCPP_INFO(this->get_logger(), "SUCCESS! Robot is now charging!");
                                result->result = result->CHARGE_SUCCESS;
                                goal_handle->succeed(result);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "FAILURE! Docking Mission succeeded, but robot is not charging.");
                                result->result = result->CHARGE_FAILED;
                                goal_handle->abort(result);
                            }
                        });
                };

                RCLCPP_INFO(this->get_logger(), "Sending docking mission goal to Nav2...");
                
                this->nav_to_pose_client_->async_send_goal(nav_goal, send_goal_options);
            }
        });
    }

    //! --- Goal 2: Handling the 'STOP_CHARGE' Goal ---

    // Check if the received goal is to stop the charging process.
    else if(goal->goal == vizzy_msgs::action::Charge::Goal::STOP_CHARGE)
    {
        RCLCPP_INFO(this->get_logger(), "Received new goal: Stop Charge! \n Leaving docking station!"); // Log message.

        geometry_msgs::msg::PoseStamped forward_pose; // Create a new NavigateToPose goal.
        forward_pose.header.frame_id = "base_footprint";  // Set the frame_id to the robot's base frame, just how Nav2 expects.
        forward_pose.header.stamp = rclcpp::Time(0); // Set the current time as the goal timestamp.

        // To move away from the docking station, we will move 1 meter forward in the robot's base frame from 
        // its current position.
        // To do this, we first define a relative pose 1 meter ahead in the "base_footprint" frame.
        forward_pose.pose.position.x = 1.0;          // Move 1 meter forward
        forward_pose.pose.orientation.w = 1.0;       // Set the orientation to a neutral quaternion (no rotation).

        // We now transform this relative pose into a static goal in the "odometry" frame.
        // We do this so the robot does not keep "chasing its own tail".
        geometry_msgs::msg::PoseStamped nav2_goal_pose;
        try
        {
            // Transform the forward pose from the "base_footprint" frame to the "odometry" frame.
            // This provides a fixed goal in the "odometry" frame, which Nav2 can then use to navigate.
            // TODO: Make the frame id a parameter.
            nav2_goal_pose = tf_buffer_->transform(forward_pose, "odometry");
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform from base_footprint to odom: %s", ex.what());
            result->result = result->STOPPED_FAILED;
            goal_handle->abort(result);
            return;
        }

        // Create the NavigateToPose goal message to send to the Nav2 action server.
        nav2_msgs::action::NavigateToPose::Goal nav2_goal;
        nav2_goal.pose = nav2_goal_pose;
        
        // Define the send goal options for the Nav2 action client.
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        // Send the pose goal to the Nav2 action server.
        send_goal_options.result_callback = [this, goal_handle, result](const auto & nav2_result)
        {
            // Check if the goal was successfully accepted and executed by the Nav2 action server.
            if (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "First step completed successfully."); // Log success message.

                // Now check the battery charging state to see if the robot is still charging.
                RCLCPP_INFO(this->get_logger(), "Checking battery state to see if the robot is still charging..."); // Log message.
                // Create a request for the battery charging state service. This service will return the current charging state of the robot.
                auto request = std::make_shared<vizzy_msgs::srv::BatteryChargingState::Request>();
                charging_state_client_->async_send_request( request,
                    [this, goal_handle, result](rclcpp::Client<vizzy_msgs::srv::BatteryChargingState>::SharedFuture response)
                    {
                        // If this is running in the simulation, set the battery state to NOT_CHARGING (0) manually.
                        // In a real robot, this would be handled by the robot's hardware and charging system.
                        if (is_simulation_)
                        {
                            RCLCPP_WARN(this->get_logger(), "Simulation mode: Forcing battery state to NOT_CHARGING after 1 meter advance.");
                            response.get()->battery_charging_state = 0; // Force NOT_CHARGING state in simulation.
                        }

                        // Check if the robot is not charging (NOT_CHARGING state is set to 0).
                        if (!response.get()->battery_charging_state)
                        {
                            result->result = result->STOPPED; // Set the action result to STOPPED (success).
                            RCLCPP_INFO(this->get_logger(), "SUCCESS! Robot is not charging anymore, therefore the undocking is completed!"); // Log success message.
                            goal_handle->succeed(result); // Inform the action client that the action was successfully completed.
                            return;
                        }
                        else // The robot is still charging (CHARGING state is set to 1).
                        {
                            result->result = result->STOPPED_FAILED; // Set the action result to STOPPED_FAILED (failure).
                            RCLCPP_INFO(this->get_logger(), "FAILURE! Robot is still charging, therefore it remains in the docking station!"); // Log error message.
                            goal_handle->abort(result); // Inform the action client that the action was aborted.
                            return;
                        }
                    });
            }
            else // The goal was not successfully accepted or executed by the Nav2 action server.
            {
                RCLCPP_INFO(this->get_logger(), "Nav2 goal failed with status: %d.", static_cast<int>(nav2_result.code)); // Log the failure status.
                result->result = result->STOPPED_FAILED; // Set the action result to STOPPED_FAILED (failure).
                goal_handle->abort(result); // Inform the action client that the action was aborted.
                return;
            }
        };

        nav_to_pose_client_->async_send_goal(nav2_goal, send_goal_options); // Send the NavigateToPose goal request to the Nav2 action server with the defined options.
    }

    //! --- Goal 3: Handling an invalid goal ---

    else // Invalid goal.
    {
        RCLCPP_INFO(this->get_logger(), "Charging action server: Invalid goal!"); // Log an error message.
        result->result = result->STOPPED_FAILED; // Set the action result to STOPPED_FAILED (failure).
        goal_handle->abort(result); // Inform the action client that the action was aborted.
    }
}

ChargingActionServer::ChargingActionServer(const rclcpp::NodeOptions & options) :
    rclcpp::Node("charging_action_server", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this))
{
    // Create publishers and service clients.
    this->cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/vizzy/cmd_vel", 1);
    this->charging_state_client_ = this->create_client<vizzy_msgs::srv::BatteryChargingState>("battery_charging_state");

    // Initialize the Nav2 action client
    this->nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose"
    );

    // Declare the parameter with a default value.
    this->declare_parameter<bool>("is_simulation", true);

    // Get the parameter's value and STORE IT in the member variable.
    is_simulation_ = this->get_parameter("is_simulation").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "is_simulation set to: %s", is_simulation_ ? "true" : "false");

    // Declare the parameter regarding the immediate activation of the dock pose detection.
    this->declare_parameter<bool>("activate_dock_pose_detection", false);

    // Get the parameter's value and store it in the member variable.
    activate_dock_pose_detection_ = this->get_parameter("activate_dock_pose_detection").as_bool();
}

void ChargingActionServer::init_action_server()
{
    this->action_server_ = rclcpp_action::create_server<vizzy_msgs::action::Charge>(
        shared_from_this(),
        "charge",
        std::bind(&ChargingActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ChargingActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ChargingActionServer::handle_accepted, this, std::placeholders::_1)
    );

    // Log that the action server has been initialized.
    RCLCPP_INFO(this->get_logger(), "Charging action server initialized.");
    RCLCPP_INFO(this->get_logger(), "Charging action server is ready to accept goals.");
    RCLCPP_INFO(this->get_logger(), "You can now send goals to the charging action server.");
    RCLCPP_INFO(this->get_logger(), "Use the 'charge' goal to start charging, or the 'stop_charge' goal to undock and stop charging.");
    RCLCPP_INFO(this->get_logger(), "Waiting for goals...");
}

rclcpp_action::GoalResponse ChargingActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const vizzy_msgs::action::Charge::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with ID: %s", rclcpp_action::to_string(uuid).c_str());
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ChargingActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal ID: %s", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ChargingActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle)
{
    // The goal is accepted, so we can now begin executing it in a new thread.
    std::thread{std::bind(&ChargingActionServer::goalCallback, this, goal_handle)}.detach();
}