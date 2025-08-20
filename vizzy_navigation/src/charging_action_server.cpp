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

    if(goal->goal == vizzy_msgs::action::Charge::Goal::CHARGE)
    {
        //* Since the charge goal, and all the process behind it, is being handled by the controller, we just need to check the battery state and return the result.

        RCLCPP_INFO(this->get_logger(), "Check battery state to make sure the robot is charging"); // Log message.
        
        //* --- Check if the robot is charging ---

        auto request = std::make_shared<vizzy_msgs::srv::BatteryChargingState::Request>(); // Create a request for the battery charging state service.
        // Send the service request to the server without blocking the current thread, crucial while using asynchronous communication.
        // The second argument is a lambda function that will be called when the service response is received.
        auto future = charging_state_client_->async_send_request(request,
            [this, goal_handle, result](rclcpp::Client<vizzy_msgs::srv::BatteryChargingState>::SharedFuture response)
        {
            // Check if the robot is charging (CHARGING state is set to 1).
            if (response.get()->battery_charging_state)
            {
                result->result = result->CHARGE_SUCCESS; // Set the action result to success.
                RCLCPP_INFO(this->get_logger(), "SUCCESS! Robot is charging!"); // Log success message.
                goal_handle->succeed(result); // The action  server informs the client that the goal has finished successfully.
                return; // End of the 'CHARGE' goal handling, exiting the function.
            }
            else // The robot is not charging (NOT_CHARGING state is set to 0).
            {
                result->result = result->CHARGE_FAILED; // Set the action result to failed.
                RCLCPP_INFO(this->get_logger(), "FAILURE! Robot is not charging!"); // Log error message.
                goal_handle->abort(result); // The action server informs the client that the goal has been aborted.
                return;
            }
        });
    }

    //! --- Goal 2: Handling the 'STOP_CHARGE' Goal ---

    // Check if the received goal is to stop the charging process.
    else if(goal->goal == vizzy_msgs::action::Charge::Goal::STOP_CHARGE)
    {
        RCLCPP_INFO(this->get_logger(), "Received new goal: Stop Charge! \n Leaving docking station!"); // Log message.

        nav2_msgs::action::NavigateToPose::Goal nav2_goal; // Create a new NavigateToPose goal.
        nav2_goal.pose.header.frame_id = "base_link";  // Set the frame_id to the robot's base frame, just how Nav2 expects.
        nav2_goal.pose.header.stamp = rclcpp::Time(0); // Set the current time as the goal timestamp.
        nav2_goal.pose.pose.position.x = 1.0;          // Move 1 meter forward
        nav2_goal.pose.pose.orientation.w = 1.0;       // Set the orientation to a neutral quaternion (no rotation).

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

        nav_client_->async_send_goal(nav2_goal, send_goal_options); // Send the NavigateToPose goal rquest to the Nav2 action server with the defined options.
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
    this->nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose"
    );
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