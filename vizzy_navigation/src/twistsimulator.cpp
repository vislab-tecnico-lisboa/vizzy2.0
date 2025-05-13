/*
    In this file, we create the node "twist_simulator", which subscribes to the topic "twist_topic",
    where teleop_twist_keyboard publishes movement commands as Twist messages.
    Upon receiving these messages, the node processes them by scaling the angular velocity and applying a deadzone to both linear and angular velocities.
    The processed Twist message is then published to the "cmd_vel" topic.
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class TwistSimulator : public rclcpp::Node
{
public:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_; // Subscriber for Twist messages
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr direct_cmd_pub_; // Publisher for processed Twist messages

  TwistSimulator()
  : Node("twist_simulator")
  {
    // Initialize the subscriber for Twist messages on 'twist_topic'
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "twist_topic", 10, std::bind(&TwistSimulator::twist_callback, this, std::placeholders::_1));
    // Initialize the publisher for processed Twist messages on 'cmd_vel'
    direct_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {

    geometry_msgs::msg::Twist direct_order = *msg; // Create a copy of the incoming Twist message

    direct_order.angular.z *= 0.45; // Scale the angular.z component, to reduce rotation speed

    // Apply deadzone thresholds to eliminate minor movements
    if (std::fabs(direct_order.angular.z) < 0.01) {
      direct_order.angular.z = 0.0;
    }
    if (std::fabs(direct_order.linear.x) < 0.01) {
      direct_order.linear.x = 0.0;
    }

    direct_cmd_pub_->publish(direct_order);  // Publish the processed Twist message to 'cmd_vel'
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistSimulator>());
  rclcpp::shutdown();
  return 0;
}
