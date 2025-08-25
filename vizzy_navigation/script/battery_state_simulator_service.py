#!/usr/bin/env python3
# filepath: /home/zenario/vizzy2_ws/src/vizzy2/vizzy_navigation/script/battery_state_simulator_service.py

import rclpy
from rclpy.node import Node
from vizzy_msgs.srv import BatteryChargingState, SetBatteryState

class BatteryChargingService(Node):
    def __init__(self):
        super().__init__('battery_charging_service')

        self.declare_parameter('initial_battery_state', 0)  # Default to 0 (NOT_CHARGING).
        
        # Initialize the battery state.
        initial_state = self.get_parameter('initial_battery_state').get_parameter_value().integer_value
        self.battery_state = initial_state
        self.get_logger().info(f"Setting initial battery state to: {self.battery_state}")

        # Service 1: Returns the current battery state
        self.get_state_srv = self.create_service(
            BatteryChargingState,
            'battery_charging_state',
            self.handle_get_state_request
        )

        # Service 2: Sets the battery state from an external call
        self.set_state_srv = self.create_service(
            SetBatteryState,
            'set_battery_state',
            self.handle_set_state_request
        )
        
        self.get_logger().info("Battery service server is ready.")

    def handle_get_state_request(self, request, response):
        """
        Handles requests to get the current battery charging state.
        The 'request' object for this service is empty.
        """
        response.battery_charging_state = self.battery_state
        self.get_logger().info(f"Received GET request. Responding with state: {self.battery_state}")
        return response

    def handle_set_state_request(self, request, response):
        """
        Handles requests to set the current battery charging state.
        The 'request' object contains a uint8 with the new state.
        """
        self.battery_state = request.battery_charging_state
        response.success = True
        self.get_logger().info(f"Received SET request. New battery state is: {self.battery_state}")
        return response

def main(args=None):
    rclpy.init(args=args)
    battery_charging_service = BatteryChargingService()
    rclpy.spin(battery_charging_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()