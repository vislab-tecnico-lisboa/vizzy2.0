#include <charging_action_server.h>
#include <vizzy_msgs/srv/battery_charging_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vizzy_msgs/action/charge.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <opennav_docking_msgs/action/dock_robot.hpp>

/***************************************************************************************************************************************************
* Function: goalCallback                                                                                            						   	   *
* Class: ChargingActionServer                                                                                             						   *
* Description: 																											  						   *
*		This function is called when a new goal is received by the action server. It handles three type of goals:								   *
*		1. CHARGE - Verifies if the robot is charging.							   				                                                   *	
*		2. STOP_CHARGE - Stops the charging process by moving the robot away from the docking station to a pre-defined pose (1 meter ahead),	   *					  
*						 and verifying if the robot is not charging.																		       *
*		3. Invalid goal - Logs an error message if the received goal is not valid.																   *				
* Parameters:                                                                                                                                      *
*		goal_handle - A shared pointer to the goal handle, which contains the goal message.                                                        *
***************************************************************************************************************************************************/

void ChargingActionServer::goalCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle)
{
    auto goal = goal_handle->get_goal(); // Get the new goal from the action client, previously accepted by the action server.
    auto result = std::make_shared<vizzy_msgs::action::Charge::Result>(); // Create a result message object to send the final result of the action to the action client.    

    //! --- Goal 1: Handling the 'CHARGE' Goal ---

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
                // The robot is already charging, so the goal is complete.
                RCLCPP_INFO(this->get_logger(), "SUCCESS! Robot is already at the dock and charging.");
                result->result = result->CHARGE_SUCCESS;
                goal_handle->succeed(result);
                return;
            }
            else
            {
                // The robot is NOT charging, so we must initiate the docking procedure with nav2.
                RCLCPP_INFO(this->get_logger(), "Robot is not charging. Initiating docking sequence...");

                // Call the docking action server.
                auto dock_goal = opennav_docking_msgs::action::DockRobot::Goal();
                dock_goal.dock_id = "dock_1"; 
                auto send_goal_options = rclcpp_action::Client<opennav_docking_msgs::action::DockRobot>::SendGoalOptions();
                send_goal_options.result_callback =
                    [this, goal_handle, result](const auto & dock_result)
                {
                    if (dock_result.code != rclcpp_action::ResultCode::SUCCEEDED)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Docking failed!");
                        result->result = result->CHARGE_FAILED;
                        goal_handle->abort(result);
                        return;
                    }

                    RCLCPP_INFO(this->get_logger(), "Docking sequence completed. Verifying charging status...");

                    // Verify that the robot is now charging.
                    auto final_request = std::make_shared<vizzy_msgs::srv::BatteryChargingState::Request>();
                    charging_state_client_->async_send_request(final_request,
                        [this, goal_handle, result](rclcpp::Client<vizzy_msgs::srv::BatteryChargingState>::SharedFuture final_response)
                        {
                            if (final_response.get()->battery_charging_state)
                            {
                                RCLCPP_INFO(this->get_logger(), "SUCCESS! Robot is now charging!");
                                result->result = result->CHARGE_SUCCESS;
                                goal_handle->succeed(result);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "FAILURE! Docking seemed to have succeeded, but robot is not charging.");
                                result->result = result->CHARGE_FAILED;
                                goal_handle->abort(result);
                            }
                        });
                };
                
                this->dock_client_->async_send_goal(dock_goal, send_goal_options);
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

        // We now transform this relative pose into a static goal in the "odom" frame.
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
    this->cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/vizzy/cmd_vel", 1);
    this->charging_state_client_ = this->create_client<vizzy_msgs::srv::BatteryChargingState>("battery_charging_state");

    // Initialize the Nav2 action client
    this->nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose"
    );

    this->dock_client_ = rclcpp_action::create_client<opennav_docking_msgs::action::DockRobot>(
        this,
        "dock"
    );

    // Declare the parameter with a default value
    this->declare_parameter<bool>("is_simulation", true);

    // Get the parameter's value and STORE IT in the member variable
    is_simulation_ = this->get_parameter("is_simulation").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "is_simulation set to: %s", is_simulation_ ? "true" : "false");
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