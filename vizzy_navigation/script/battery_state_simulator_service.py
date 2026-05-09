#!/usr/bin/env python3
#
# Copyright 2026, João Penha Lopes. All rights reserved.
#
# Battery state simulator for Vizzy simulation.
#
# Serves two read services and one write service:
#
#   battery_charging_state (vizzy_msgs/srv/BatteryChargingState)
#   - Used by the charging/docking system.
#
#   battery_state (vizzy_msgs/srv/BatteryState)
#   - Used by patrol_node. Response includes battery_state (int) and
#     percentage (float). States: 0=CHARGED, 1=GOOD, 2=UNKNOWN, 3=LOW_BATTERY.
#
#   set_battery_state (vizzy_msgs/srv/SetBatteryState)
#   - External trigger. Call with battery_charging_state=3 to simulate
#     a low-battery event and interrupt the patrol run:
#
#       ros2 service call /set_battery_state vizzy_msgs/srv/SetBatteryState "{battery_charging_state: 3}"
#
#   Reset to GOOD (1) after the charge cycle completes:
#
#       ros2 service call /set_battery_state vizzy_msgs/srv/SetBatteryState "{battery_charging_state: 1}"

import rclpy
from rclpy.node import Node
from vizzy_msgs.srv import BatteryChargingState, BatteryState, SetBatteryState

# Approximate percentage reported for each battery state.
_STATE_PERCENTAGE = {0: 100.0, 1: 80.0, 2: 50.0, 3: 10.0}

class BatterySimulatorService(Node):

    def __init__(self):
        super().__init__('battery_charging_service')

        self.declare_parameter('initial_battery_state', 1)  # Default: GOOD.

        initial_state = self.get_parameter('initial_battery_state').get_parameter_value().integer_value
        self.battery_state = initial_state
        self.get_logger().info(f'Initial battery state: {self.battery_state}')

        self.create_service(BatteryChargingState, 'battery_charging_state',
                            self._handle_get_charging_state)
        self.create_service(BatteryState,         'battery_state',
                            self._handle_get_battery_state)
        self.create_service(SetBatteryState,      'set_battery_state',
                            self._handle_set_state)

        self.get_logger().info('Battery simulator ready. '
                               'Call /set_battery_state with state=3 to trigger low-battery.')

    def _handle_get_charging_state(self, request, response):
        response.battery_charging_state = self.battery_state
        return response

    def _handle_get_battery_state(self, request, response):
        response.battery_state = self.battery_state
        response.percentage    = _STATE_PERCENTAGE.get(self.battery_state, 50.0)
        return response

    def _handle_set_state(self, request, response):
        old = self.battery_state
        self.battery_state = request.battery_charging_state
        response.success = True
        self.get_logger().info(f'Battery state changed: {old} -> {self.battery_state} '
                               f'({_STATE_PERCENTAGE.get(self.battery_state, 50.0):.0f}%)')
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatterySimulatorService())
    rclpy.shutdown()


if __name__ == '__main__':
    main()