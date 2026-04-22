#!/usr/bin/env python3

# Copyright 2025, João Penha Lopes. All rights reserved.
#
# Migrated from ROS1 vizzy_serial_interfaces (battery-state-service.cpp and
# battery-charging-state-service.cpp) by João Penha Lopes.
#
# Provides two real-hardware-backed services sourced from the Kokam battery
# serial PCB.

import collections
import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from vizzy_msgs.srv import BatteryChargingState, BatteryState


class BatteryKokamService(Node):
    """Reads voltage and current from the Kokam battery serial PCB and exposes
    two ROS 2 services: battery_state and battery_charging_state.

    A background thread polls the serial port continuously so that both service
    handlers return immediately without blocking on I/O.
    """

    # Voltage range for percentage calculation, matches ROS1 hardcoded values.
    _MIN_VOLTAGE = 22.0
    _MAX_VOLTAGE = 29.4

    def __init__(self):
        super().__init__('battery_kokam_service')

        # Parameters (defaults match ROS1 battery-state.launch).
        self.declare_parameter('port', '/dev/kokam_power')
        self.declare_parameter('charged_thr', 29.39)
        self.declare_parameter('medium_thr', 25.5)
        self.declare_parameter('low_thr', 24.5)

        # Number of samples used for slope calculation and the minimum required
        # before reporting a charging state, both set to 40 to match the ROS1
        # battery-charging-state-service.cpp (samples = 40).
        self.declare_parameter('slope_window', 40)
        self.declare_parameter('min_slope_samples', 40)

        port = self.get_parameter('port').value
        self._charged_thr = self.get_parameter('charged_thr').value
        self._medium_thr = self.get_parameter('medium_thr').value
        self._low_thr = self.get_parameter('low_thr').value
        slope_window = self.get_parameter('slope_window').value
        self._min_slope_samples = self.get_parameter('min_slope_samples').value

        # Thread-safe sample buffer.
        self._lock = threading.Lock()
        # maxlen matches slope_window so the deque acts as a rolling window.
        self._samples: collections.deque = collections.deque(maxlen=slope_window)
        self._latest_voltage: float | None = None

        # Serial port setup.
        try:
            self._serial = serial.Serial(port, baudrate=115200, timeout=1)
            self.get_logger().info(f'{port} open at 115200 baud')
        except serial.SerialException as exc:
            self.get_logger().fatal(f'Cannot open serial port {port}: {exc}')
            raise

        # Background reader.
        reader = threading.Thread(target=self._read_loop, daemon=True)
        reader.start()

        # Services.
        self.create_service(BatteryState, 'battery_state', self._handle_battery_state)
        self.create_service(
            BatteryChargingState, 'battery_charging_state', self._handle_battery_charging_state
        )

        self.get_logger().info('Battery Kokam service ready.')

    # ------------------------------------------------------------------
    # Serial reader (background thread)
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        """Continuously reads voltage and current from the Kokam PCB.

        Protocol (mirrors VoltageCurrentSerialInterface::getSystemPowerSupply):
          1. Write byte '0'
          2. Sleep 300 ms
          3. Read 8 bytes
          4. Sleep 300 ms
          5. Validate: buf[0] == '0', checksum(buf[0:6]) == buf[6:8] as uint16 BE
          6. Decode: voltage = (buf[1]<<8 | buf[2]) / 100.0
                     current = (buf[3]<<8 | buf[4]) / 100.0
        """
        while rclpy.ok():
            try:
                self._serial.write(b'0')
                time.sleep(0.3)
                buf = self._serial.read(8)
                time.sleep(0.3)

                if len(buf) != 8 or buf[0] != ord('0'):
                    self.get_logger().warning(
                        f'Unexpected response: {len(buf)} bytes, '
                        f'first=0x{buf[0]:02x}' if buf else 'empty'
                    )
                    continue

                byte_sum = sum(buf[:6])
                checksum = (buf[6] << 8) | buf[7]
                if checksum != byte_sum:
                    self.get_logger().warning('Checksum mismatch, sample discarded.')
                    continue

                voltage = ((buf[1] << 8) | buf[2]) / 100.0
                # current is decoded but unused for state logic (unsigned, matches ROS1).
                # current = ((buf[3] << 8) | buf[4]) / 100.0

                with self._lock:
                    self._samples.append((time.monotonic(), voltage))
                    self._latest_voltage = voltage

            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                time.sleep(1.0)

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _classify_voltage(self, voltage: float) -> int:
        """Maps a voltage reading to a BatteryState constant.

        Mirrors battery_state_logic() in battery-state-service.cpp exactly.
        """
        R = BatteryState.Response
        if voltage > self._charged_thr:
            return R.CHARGED
        if voltage > self._medium_thr:
            return R.GOOD
        if voltage > self._low_thr:
            return R.MEDIUM
        return R.LOW_BATTERY

    def _handle_battery_state(
        self, request: BatteryState.Request, response: BatteryState.Response
    ) -> BatteryState.Response:
        with self._lock:
            voltage = self._latest_voltage

        if voltage is None:
            response.battery_state = BatteryState.Response.UNKNOWN
            response.percentage = 101  # sentinel used in ROS1 on read failure.
        else:
            response.battery_state = self._classify_voltage(voltage)
            pct = 100.0 * (voltage - self._MIN_VOLTAGE) / (self._MAX_VOLTAGE - self._MIN_VOLTAGE)
            response.percentage = int(math.floor(pct))

        self.get_logger().info(
            f'battery_state → state={response.battery_state} percentage={response.percentage}'
        )
        return response

    @staticmethod
    def _l2_slope(samples: list) -> float:
        """Least-squares slope (V/s) of (time, voltage) pairs.

        Mirrors l2_line_fitting() in battery-charging-state-service.cpp.
        Returns 0.0 when the denominator is zero (all timestamps identical).
        """
        n = len(samples)
        t_vals = [s[0] for s in samples]
        v_vals = [s[1] for s in samples]
        t_sum = sum(t_vals)
        v_sum = sum(v_vals)
        t2_sum = sum(t * t for t in t_vals)
        tv_sum = sum(t * v for t, v in zip(t_vals, v_vals))
        denom = n * t2_sum - t_sum * t_sum
        if denom == 0.0:
            return 0.0
        return (n * tv_sum - t_sum * v_sum) / denom

    def _handle_battery_charging_state(
        self,
        request: BatteryChargingState.Request,
        response: BatteryChargingState.Response,
    ) -> BatteryChargingState.Response:
        with self._lock:
            samples = list(self._samples)

        R = BatteryChargingState.Response
        if len(samples) < self._min_slope_samples:
            response.battery_charging_state = R.UNKNOWN
        else:
            slope = self._l2_slope(samples)
            self.get_logger().info(f'battery_charging_state → slope={slope:.6f} V/s')
            response.battery_charging_state = R.CHARGING if slope > 0.0 else R.NOT_CHARGING

        return response

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        if self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryKokamService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
