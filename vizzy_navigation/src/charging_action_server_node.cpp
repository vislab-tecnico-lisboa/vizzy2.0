#include "charging_action_server.h"

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library.
    rclcpp::init(argc, argv);

    // Create an instance of the node using a shared pointer.
    // We are creating the node with default options.
    auto action_server = std::make_shared<ChargingActionServer>(rclcpp::NodeOptions());

    action_server->init_action_server();
    
    // Use a MultiThreadedExecutor to run the node.
    // This is the ROS 2 equivalent of the MultiThreadedSpinner from ROS 1.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);

    // Start the executor to process callbacks. This will block until the node is shut down.
    executor.spin();

    // Shut down the ROS 2 client library.
    rclcpp::shutdown();
    
    return 0;
}