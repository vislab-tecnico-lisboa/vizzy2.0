/* 
 * Copyright 2025, João Avelino, Rui Figueiredo, Plinio Moreno, João Penha Lopes and João Zenário.
 * This code was originaly developped by João Avelino, Rui Figueiredo and Plinio Moreno for ROS1 in 2022.
 * It was later refactored for ROS2 in 2025 by João Penha Lopes and João Zenário.
 * All rights reserved.
 */

#ifndef CHARGING_ACTION_SERVER_H_
#define CHARGING_ACTION_SERVER_H_

#include <memory>
#include <string>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include "vizzy_msgs/action/charge.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "vizzy_msgs/srv/battery_charging_state.hpp"
#include <opennav_docking_msgs/action/dock_robot.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @brief A ROS 2 action server for managing charging behavior.
 */
class ChargingActionServer : public rclcpp::Node
{
public:
    
    /**
     * @brief Constructor for the ChargingActionServer class.
     * This is a declaration. The implementation is in the .cpp file.
     * It takes the node options as a parameter.
     * @param options The node options for configuring the ROS 2 node.
     */
    ChargingActionServer(const rclcpp::NodeOptions & options);

    /**
     * @brief A method to initialize the action server.
     * It creates the action server and registers the goal, cancel, and accepted callbacks.
     * @return void.
     */
    void init_action_server();
    
private:

    /**
     * @brief The action server for handling charging requests.
     */
    rclcpp_action::Server<vizzy_msgs::action::Charge>::SharedPtr action_server_;
    
    /**
     * @brief The publisher for sending velocity commands to the topic.
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    /**
     * @brief The client for requesting battery charging state.
     */
    rclcpp::Client<vizzy_msgs::srv::BatteryChargingState>::SharedPtr charging_state_client_;
    
    /**
     * @brief The client for requesting navigation to a pose.
     * This client will send the navigation goal to the bt_navigator that will either:
     * 
     * - Use a custom BT to execute the docking procedure.
     * 
     * - Navigate 1 meter forward to exit the dock.
     */
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;

    /**
     * @brief Parameter to indicate if the action server is running in a simulation or on a real robot.
     * This is used to autonomously activate or not the battery state simulator used by the
     * ChargingActionServer.
     */
    bool is_simulation_;

    /**
     * @brief TF2 buffer for storing transforms.
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /**
     * @brief TF2 listener for receiving transform updates.
     */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * @brief Boolean flag to indicate if the user wants to activate the dock pose detection immediately.
     */
    bool activate_dock_pose_detection_;
    
    /**
     * @brief The goal callback for the action server.
     * This is the main processing function for the action server.
     * This function is called when a new goal is received by the action server. It handles three type of goals:	
     * 							   
     *	1. CHARGE - Verifies if the robot is charging.					
     *		   				                                                   	
     *	2. STOP_CHARGE - Stops the charging process by moving the robot away from the docking station to a pre-defined pose (1 meter ahead),	  				  
     *	and verifying if the robot is not charging.				
     *														       
     *	3. Invalid goal - Logs an error message if the received goal is not valid.	
     * @return void.
     */
    void goalCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle);

    /**
     * @brief Handles a new goal request.
     * If the goal is accepted, it will call the handle_accepted method with the goal.
     * @param uuid The unique identifier for the goal.
     * @param goal The goal request.
     * @return The response indicating whether the goal is accepted.
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const vizzy_msgs::action::Charge::Goal> goal);


    /**
     * @brief Handles a cancel request.
     * @param goal_handle The goal handle for the action.
     * @return The response indicating whether the cancel request is accepted.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle);
    
    
    /**
     * @brief Handles an accepted goal request.
     * @param goal_handle The goal handle for the action.
     * @return void.
     */
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle);
};

#endif // CHARGING_ACTION_SERVER_H_