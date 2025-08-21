#ifndef CHARGING_ACTION_SERVER_H_
#define CHARGING_ACTION_SERVER_H_

#include <memory>
#include <string>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS 2 TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ROS 2 standard messages
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Custom ROS 2 messages
#include <vizzy_msgs/action/charge.hpp>
#include <vizzy_msgs/srv/battery_charging_state.hpp>

// Nav2 action client
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <opennav_docking_msgs/action/dock_robot.hpp>

class ChargingActionServer : public rclcpp::Node
{
public:
    // ROS 2 standard constructor
    explicit ChargingActionServer(const rclcpp::NodeOptions & options);
    void init_action_server();
    
private:
    // Pointers for ROS 2 action server and communication objects
    rclcpp_action::Server<vizzy_msgs::action::Charge>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<vizzy_msgs::srv::BatteryChargingState>::SharedPtr charging_state_client_;
    
    // New: Nav2 action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<opennav_docking_msgs::action::DockRobot>::SharedPtr dock_client_;


    // TF2 members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // The main goal execution logic is now in one function
    void goalCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle);

    // ROS 2 Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const vizzy_msgs::action::Charge::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vizzy_msgs::action::Charge>> goal_handle);
};

#endif // CHARGING_ACTION_SERVER_H_