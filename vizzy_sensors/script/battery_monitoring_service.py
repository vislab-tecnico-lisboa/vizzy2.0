#!/usr/bin/env python3
#
# Copyright 2026, João Penha Lopes. All rights reserved.
#
# Migrated from ROS1 vizzy_serial_interfaces (battery-state-service.cpp and
# battery-charging-state-service.cpp) by João Penha Lopes.
#
# Provides two real-hardware-backed services sourced from the Kokam battery
# serial PCB: battery_state and battery_charging_state.
#
# ------------------------------------------------------------------------------
# Design notes: differences from ROS1 and why this version is better
# ------------------------------------------------------------------------------
#
# ROS1 architecture (battery-state-service.cpp,
#                    battery-charging-state-service.cpp)
# ------------------------------------------------------
# Both services were blocking:
#
#   battery_state        , on each service call, performed one serial
#                           round-trip (write '0', sleep 300 ms, read 8 bytes,
#                           sleep 300 ms) and returned immediately. Latency
#                           per call: ~600 ms.
#
#   battery_charging_state, on each service call, performed 40 sequential
#                           round-trips (samples = 40), collecting
#                           (timestamp, voltage) pairs, then fitted a linear
#                           slope with L2 least squares. Total latency per
#                           call: 40 × 600 ms ≈ 24 seconds. The ROS service
#                           thread was blocked for the entire collection period.
#                           Classification: slope > 0 → CHARGING,
#                                           slope ≤ 0 → NOT_CHARGING.
#
# ROS2 architecture (this file)
# -----------------------------
# A single background thread polls the serial port continuously at the same
# ~600 ms/sample rate and maintains a rolling deque of the last `slope_window`
# (default 40) samples. Both service handlers read from this shared buffer and
# return immediately. This eliminates the 24-second blocking call while
# preserving the same sampling protocol and slope estimation as ROS1, 
# while using a more robust charging-state decision rule for the continuously 
# updated ROS2 architecture.
#
# Charging-state noise problem and hysteresis solution
# ----------------------------------------------------
# The voltage is decoded as a 16-bit unsigned integer divided by 100, giving
# a resolution of 0.01 V per LSB. When the battery is static (no charger, no
# load change), the true voltage is flat, but individual readings fluctuate by
# ±1 LSB due to ADC quantisation noise. Over a 40-sample / 24-second window
# this produces a spurious slope of approximately:
#
#   noise slope ≈ ΔV / Δt = 0.01 V / 24 s ≈ 0.0004 V/s
#
# In ROS1 this was not observable in practice because the 24-second collection
# was triggered on demand (typically infrequently). In this ROS2 port the
# rolling window is continuously updated, so consecutive service calls see
# overlapping sample sets whose slope oscillates around zero at the noise
# magnitude above, causing false CHARGING / NOT_CHARGING transitions on a
# static, uncharged battery.
#
# The solution we decided to apply in this case is hysteresis: 
# the threshold to enter a state differs from the threshold to
# exit it, and in the absence of a clear signal the system holds its last
# known state. This node implements symmetric hysteresis with a single
# `slope_threshold` parameter (default 0.001 V/s, ~2.5× the noise floor):
#
#   current state = NOT_CHARGING:
#     slope >  +slope_threshold  →  transition to CHARGING
#     slope ≤  +slope_threshold  →  remain NOT_CHARGING
#
#   current state = CHARGING:
#     slope < −slope_threshold   →  transition to NOT_CHARGING
#     slope ≥ −slope_threshold   →  remain CHARGING
#
# A real charging event is expected to produce a sustained positive slope well
# above the quantisation-noise floor, however the threshold should still be
# validated on the hardware (TODO). Quantisation noise, which has no dominant
# direction, never exceeds the threshold in a sustained way and cannot trigger
# a false transition. The service always returns a definite CHARGING or
# NOT_CHARGING, including during the startup window and after a serial port
# reopen, where the hysteresis state (initialised to NOT_CHARGING) is returned
# instead.
#
# Serial port resilience
# ----------------------
# USB-to-serial devices can undergo kernel-level resets (autosuspend, power
# glitch, udev re-trigger) that invalidate the open file descriptor. The read
# loop detects consecutive SerialException errors and attempts to reopen the
# port after a short back-off, clearing the sample buffer so that stale
# data spanning the error gap cannot distort the slope estimate.
#
# Notes: before figuring out that the issue of the constant USB-resetting was
# due to the USB hub, I tried turning autosuspend off:
# vizzy@vizzy-System-Product-Name:~$ cat /etc/udev/rules.d/99-kokam-battery.rules
# SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="Vizzy_BAT", SYMLINK+="kokam_power", MODE="0666"
# SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="Vizzy_BAT", ATTR{power/control}="on"
# Shouldn't affect anything else though.


import collections
import math
import termios
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from vizzy_msgs.msg import BatterySource, BatteryStatus
from vizzy_msgs.srv import BatteryChargingState, BatteryState
from segway_rmp_ros2.msg import SegwayStatusStamped


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
        super().__init__('battery_monitoring_service')

        # Parameters (defaults match ROS1 battery-state.launch).
        self.declare_parameter('port', '/dev/kokam_power')
        self.declare_parameter('charged_thr', 27.92)   # ~80% of the 22-29.4 V range.
        self.declare_parameter('medium_thr', 26.81)    # ~65%: GOOD/MEDIUM boundary, kept between low and charged.
        self.declare_parameter('low_thr', 25.70)       # ~50% of the 22-29.4 V range.

        # Number of samples used for slope calculation and the minimum required
        # before reporting a charging state, both set to 40 to match the ROS1
        # battery-charging-state-service.cpp (samples = 40).
        self.declare_parameter('slope_window', 40)
        self.declare_parameter('min_slope_samples', 40)

        # Hysteresis dead-band: slope must exceed this magnitude (V/s) to
        # trigger a state transition. Default is ~2.5× the ADC quantisation
        # noise floor (~0.0004 V/s) observed on this hardware.
        self.declare_parameter('slope_threshold', 0.001)

        # Optional idle gap between query cycles (seconds). Set to 0.0 for
        # the tightest ROS1-equivalent polling rate (~600 ms/sample, 40 samples
        # fills the slope window in ~24 s).
        self.declare_parameter('inter_cycle_delay', 0.0)

        # Kokam derives is_low / is_full from its voltage thresholds above
        # (low = low_thr, full = charged_thr). The two Segway batteries are
        # reported by the driver as VOLTAGES; each is converted to a state-of-charge
        # % over its operating range and flagged low/full at segway_low_pct /
        # segway_full_pct (20% / 80% == 20% above min, 20% below max).
        self.declare_parameter('segway_ui_min_voltage', 5.5)          # V
        self.declare_parameter('segway_ui_max_voltage', 9.1)          # V
        self.declare_parameter('segway_powerbase_min_voltage', 60.0)  # V
        self.declare_parameter('segway_powerbase_max_voltage', 68.0)  # V
        self.declare_parameter('segway_low_pct', 20.0)                # is_low  at/below this SOC %
        self.declare_parameter('segway_full_pct', 80.0)               # is_full at/above this SOC %
        self.declare_parameter('status_publish_rate', 1.0)            # Hz

        port = self.get_parameter('port').value
        self._charged_thr = self.get_parameter('charged_thr').value
        self._medium_thr = self.get_parameter('medium_thr').value
        self._low_thr = self.get_parameter('low_thr').value
        slope_window = self.get_parameter('slope_window').value
        self._min_slope_samples = self.get_parameter('min_slope_samples').value
        self._slope_threshold = self.get_parameter('slope_threshold').value
        self._inter_cycle_delay = self.get_parameter('inter_cycle_delay').value
        self._segway_ui_min_v = self.get_parameter('segway_ui_min_voltage').value
        self._segway_ui_max_v = self.get_parameter('segway_ui_max_voltage').value
        self._segway_pb_min_v = self.get_parameter('segway_powerbase_min_voltage').value
        self._segway_pb_max_v = self.get_parameter('segway_powerbase_max_voltage').value
        self._segway_low_pct = self.get_parameter('segway_low_pct').value
        self._segway_full_pct = self.get_parameter('segway_full_pct').value
        status_publish_rate = self.get_parameter('status_publish_rate').value

        # Thread-safe sample buffer.
        self._lock = threading.Lock()
        # maxlen matches slope_window so the deque acts as a rolling window.
        self._samples: collections.deque = collections.deque(maxlen=slope_window)
        self._latest_voltage: float | None = None
        self._latest_current: float | None = None
        self._latest_slope: float = 0.0

        # Latest Segway battery readings from /segway_status (None until first msg).
        self._segway_ui_voltage: float | None = None
        self._segway_pb_value: float | None = None

        # Hysteresis state: persists between service calls.
        # Initialised to NOT_CHARGING and matches initial_battery_state=0 used
        # by the simulator which is correct when the charger is disconnected.
        self._current_charging_state: int = BatteryChargingState.Response.NOT_CHARGING

        # Serial port setup.
        # dsrdtr=False / rtscts=False prevents pyserial from asserting DTR/RTS
        # on open. Trying to fix infinite reset loop.
        try:
            self._serial = serial.Serial(
                port, baudrate=115200, timeout=1, dsrdtr=False, rtscts=False
            )
            self._serial.dtr = False
            self.get_logger().info(f'{port} open at 115200 baud')
        except serial.SerialException as exc:
            self.get_logger().fatal(f'Cannot open serial port {port}: {exc}')
            raise

        # Background reader.
        reader = threading.Thread(target=self._read_loop, daemon=True)
        reader.start()

        # Services (retained for backward compatibility, Kokam only).
        self.create_service(BatteryState, 'battery_state', self._handle_battery_state)
        self.create_service(
            BatteryChargingState, 'battery_charging_state', self._handle_battery_charging_state
        )

        # Aggregated multi-source battery status (Kokam + both Segway batteries).
        self.create_subscription(
            SegwayStatusStamped, '/segway_status', self._on_segway_status, 10)
        self._status_pub = self.create_publisher(BatteryStatus, 'battery_status', 10)
        period = 1.0 / status_publish_rate if status_publish_rate > 0.0 else 1.0
        self.create_timer(period, self._publish_status)

        self.get_logger().info(
            'Battery monitor ready (Kokam + Segway; publishing /battery_status).')

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

        On persistent serial errors the port is closed and reopened. The sample
        buffer is cleared on reopen so that the time gap does not corrupt the
        slope estimate.
        """
        consecutive_errors = 0
        max_consecutive_errors = 3

        while rclpy.ok():
            try:
                self._serial.reset_input_buffer()
                self._serial.write(b'0')
                time.sleep(0.3)
                buf = self._serial.read(8)
                time.sleep(0.3)

                consecutive_errors = 0  # reset on any successful I/O.

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
                # current is unsigned (magnitude, matches ROS1); the PCB exposes no
                # sign/charge bit, so it is reported as-is for visualization only and
                # is NOT used for charging detection (that stays the voltage slope).
                current = ((buf[3] << 8) | buf[4]) / 100.0

                with self._lock:
                    self._samples.append((time.monotonic(), voltage))
                    self._latest_voltage = voltage
                    self._latest_current = current

                if self._inter_cycle_delay > 0.0:
                    time.sleep(self._inter_cycle_delay)

            except (serial.SerialException, termios.error) as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                consecutive_errors += 1
                time.sleep(1.0)

                if consecutive_errors >= max_consecutive_errors:
                    self.get_logger().warning(
                        f'Attempting to reopen {self._serial.port} after '
                        f'{consecutive_errors} consecutive errors...'
                    )
                    try:
                        self._serial.close()
                        # Wait for the USB-serial bridge to finish its
                        # re-enumeration cycle before attempting reopen.
                        time.sleep(2.0)
                        self._serial.open()
                        # Discard any garbage left in the FIFO from before
                        # the reset, then wait for the line to settle.
                        self._serial.reset_input_buffer()
                        time.sleep(0.5)
                        with self._lock:
                            self._samples.clear()
                        consecutive_errors = 0
                        self.get_logger().info(
                            f'{self._serial.port} reopened successfully.'
                        )
                    except (serial.SerialException, OSError) as reopen_exc:
                        self.get_logger().error(f'Reopen failed: {reopen_exc}')

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

    def _compute_charging_state(self) -> int:
        """Update and return the Kokam charging state via the slope hysteresis.

        Shared by the battery_charging_state service and the status publisher so
        the charging decision uses one consistent state machine. Stores the most
        recent slope in self._latest_slope for reporting. Unchanged detection
        logic from before (voltage slope + hysteresis).
        """
        with self._lock:
            samples = list(self._samples)

        R = BatteryChargingState.Response

        if len(samples) < self._min_slope_samples:
            return self._current_charging_state

        slope = self._l2_slope(samples)
        self._latest_slope = slope

        if self._current_charging_state == R.NOT_CHARGING:
            if slope > self._slope_threshold:
                self._current_charging_state = R.CHARGING
        else:  # currently CHARGING
            if slope < -self._slope_threshold:
                self._current_charging_state = R.NOT_CHARGING

        return self._current_charging_state

    def _handle_battery_charging_state(
        self,
        request: BatteryChargingState.Request,
        response: BatteryChargingState.Response,
    ) -> BatteryChargingState.Response:
        state = self._compute_charging_state()
        self.get_logger().info(
            f'battery_charging_state → slope={self._latest_slope:.6f} V/s state={state}'
        )
        response.battery_charging_state = state
        return response

    # ------------------------------------------------------------------
    # Segway batteries + aggregated status
    # ------------------------------------------------------------------

    def _on_segway_status(self, msg: SegwayStatusStamped) -> None:
        with self._lock:
            self._segway_ui_voltage = float(msg.segway.ui_battery)
            self._segway_pb_value = float(msg.segway.powerbase_battery)

    def _make_segway_source(self, name, voltage, vmin, vmax, charging_state):
        """Build a BatterySource for a Segway battery (reported as a voltage).

        Converts voltage to an SOC % over [vmin, vmax] and flags low/full at the
        configured SOC thresholds. No reading -> level UNKNOWN, not low, not full
        (a missing source never forces a dock nor lets the robot undock blindly).
        """
        src = BatterySource()
        src.name = name
        src.charging_state = charging_state
        if voltage is None:
            src.level = BatterySource.LEVEL_UNKNOWN
            return src
        pct = 100.0 * (voltage - vmin) / (vmax - vmin)
        pct = max(0.0, min(100.0, pct))
        src.has_voltage = True
        src.voltage = voltage
        src.has_percentage = True
        src.percentage = pct
        src.is_low = pct <= self._segway_low_pct
        src.is_full = pct >= self._segway_full_pct
        src.level = self._coarse_level(pct, src.is_low, src.is_full)
        return src

    @staticmethod
    def _coarse_level(value, is_low, is_full):
        if value is None:
            return BatterySource.LEVEL_UNKNOWN
        if is_low:
            return BatterySource.LOW_BATTERY
        if is_full:
            return BatterySource.CHARGED
        return BatterySource.GOOD

    def _publish_status(self) -> None:
        # All sources charge through one connector, so charging is global.
        charging_state = self._compute_charging_state()
        charging = (charging_state == BatteryChargingState.Response.CHARGING)

        with self._lock:
            v = self._latest_voltage
            i = self._latest_current
            slope = self._latest_slope
            ui_v = self._segway_ui_voltage
            pb = self._segway_pb_value

        # Kokam (voltage + current + derived %, with the final voltage thresholds).
        kokam = BatterySource()
        kokam.name = 'kokam'
        kokam.has_voltage = v is not None
        kokam.has_current = i is not None
        kokam.has_percentage = v is not None
        kokam.charge_slope = slope
        kokam.charging_state = charging_state
        if i is not None:
            kokam.current = i
        if v is not None:
            kokam.voltage = v
            pct = 100.0 * (v - self._MIN_VOLTAGE) / (self._MAX_VOLTAGE - self._MIN_VOLTAGE)
            kokam.percentage = float(max(0.0, min(100.0, pct)))
            kokam.level = self._classify_voltage(v)
            kokam.is_low = v <= self._low_thr
            kokam.is_full = v >= self._charged_thr
        else:
            kokam.level = BatterySource.LEVEL_UNKNOWN
            # No Kokam reading -> conservatively NOT full, so the robot never
            # undocks blindly when the primary battery is unreadable.
            kokam.is_full = False

        # Segway batteries are reported as voltages; convert to SOC % over each
        # battery's operating range (see _make_segway_source).
        ui = self._make_segway_source(
            'segway_ui', ui_v,
            self._segway_ui_min_v, self._segway_ui_max_v, charging_state)
        pbs = self._make_segway_source(
            'segway_powerbase', pb,
            self._segway_pb_min_v, self._segway_pb_max_v, charging_state)

        # Aggregate
        msg = BatteryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kokam = kokam
        msg.segway_ui = ui
        msg.segway_powerbase = pbs
        msg.charging = charging
        msg.any_low = kokam.is_low or pbs.is_low # or ui.is_low (ui is not accounted for, very reactive/unimportant)
        msg.all_full = kokam.is_full and pbs.is_full # ui.is_full (ui is not accounted for, very reactive/unimportant)

        alerts = []
        if kokam.is_low:
            alerts.append(f'kokam low ({v:.2f} V)')
        #if ui.is_low: (ui is not accounted for, very reactive/unimportant)
        #    alerts.append(f'segway_ui low ({ui_v:.2f} V, {ui.percentage:.0f}%)')
        if pbs.is_low:
            alerts.append(f'segway_powerbase low ({pb:.2f} V, {pbs.percentage:.0f}%)')
        if v is None:
            alerts.append('kokam: no serial reading')
        if ui_v is None:
            alerts.append('segway: no /segway_status received')
        msg.alerts = alerts

        if msg.any_low:
            msg.system_status = BatteryStatus.CRITICAL
        elif (not msg.all_full) or v is None or ui_v is None:
            msg.system_status = BatteryStatus.WARN
        else:
            msg.system_status = BatteryStatus.OK

        self._status_pub.publish(msg)

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
