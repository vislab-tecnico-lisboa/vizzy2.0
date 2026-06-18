#!/usr/bin/env python3
#
# Copyright 2026, João Penha Lopes. All rights reserved.
#
# Recovery supervisor for Vizzy.
#
# A standalone, behavior-agnostic node that:
# monitors localization health (AMCL pose covariance),
# attempts automatic relocalization (re-seed AMCL to the last-known-good
# pose + spin in place to re-converge) when localization diverges, useful
# when the map is stale (e.g. tables/chairs moved) and AMCL drifts,
# owns a single system recovery state (OK / RECOVERING / FAULT) and
# broadcasts it on /recovery_status so ANY behavior (patrol, etc.) can react,
# accepts unrecoverable-fault reports from other nodes (/report_fault) and
# latches FAULT until a human clears it (/clear_fault).
#
# Contract: while state != OK, consumers MUST stop commanding the base (cancel
# their navigation goals). During RECOVERING the supervisor itself drives the
# base (spin) to relocalize; during FAULT the robot stays put.

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import Spin
from vizzy_msgs.msg import RecoveryStatus


class RecoverySupervisor(Node):

    def __init__(self):
        super().__init__('recovery_supervisor')

        # --- Parameters ---
        # Thresholds calibrated from on-robot /amcl_pose data. Good fixes stay below
        # ~0.0036 (position: var x + var y) and ~0.0013 (yaw var); wrong/uncertain
        # fixes exceed the diverged values. Position and yaw are OR-combined for
        # divergence (yaw catches early drift the position term misses).
        # NOTE: covariance is *confidence*, not *correctness*, meaning a confidently-wrong
        # fix has LOW covariance and is NOT detectable here; those are caught
        # downstream (planning failures / whole-run-missed -> FAULT).
        self.declare_parameter('good_covariance', 0.0035)         # pos var x+y: store last-good below this
        self.declare_parameter('good_yaw_covariance', 0.0013)     # yaw var:     store last-good below this
        self.declare_parameter('diverged_covariance', 0.015)      # pos var x+y: divergence above this
        self.declare_parameter('diverged_yaw_covariance', 0.0028) # yaw var:     divergence above this
        self.declare_parameter('divergence_time', 4.0)            # s sustained above threshold
        # Honor an externally-set pose on /initialpose (e.g. RViz): adopt it and
        # pause divergence checks for this long so AMCL converges without an auto re-seed.
        self.declare_parameter('manual_pose_grace_period', 2.0)  # s
        self.declare_parameter('max_relocalize_attempts', 2)
        self.declare_parameter('relocalize_spin_yaw', 6.28)    # rad (~full turn) to sweep features
        self.declare_parameter('settle_time', 1.5)             # s for consumers to stop before spin
        self.declare_parameter('post_spin_settle', 1.5)        # s for AMCL to re-converge after spin
        self.declare_parameter('monitor_period', 0.5)          # s

        self._good_cov = self.get_parameter('good_covariance').value
        self._good_yaw_cov = self.get_parameter('good_yaw_covariance').value
        self._diverged_cov = self.get_parameter('diverged_covariance').value
        self._diverged_yaw_cov = self.get_parameter('diverged_yaw_covariance').value
        self._divergence_time = self.get_parameter('divergence_time').value
        self._manual_grace = self.get_parameter('manual_pose_grace_period').value
        self._max_attempts = self.get_parameter('max_relocalize_attempts').value
        self._spin_yaw = self.get_parameter('relocalize_spin_yaw').value
        self._settle_time = self.get_parameter('settle_time').value
        self._post_spin_settle = self.get_parameter('post_spin_settle').value
        monitor_period = self.get_parameter('monitor_period').value

        # --- State ---
        self._state = RecoveryStatus.OK
        self._reason = 'nominal'
        self._state_lock = threading.Lock()
        self._last_good_pose = None    # PoseWithCovarianceStamped
        self._latest_cov = None        # float, var(x) + var(y)
        self._latest_yaw_cov = None    # float, var(yaw)
        self._diverged_since = None    # monotonic timestamp
        self._grace_until = 0.0        # monotonic; suppress divergence checks until then
        self._recovering = threading.Event()  # guards the relocalization worker

        # --- Interfaces ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(RecoveryStatus, 'recovery_status', latched)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, 10)
        # Honor externally-set poses (RViz "2D Pose Estimate" -> /initialpose).
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self._on_initialpose, 10)
        self._spin_client = ActionClient(self, Spin, 'spin')
        self.create_service(Trigger, 'report_fault', self._on_report_fault)
        self.create_service(Trigger, 'clear_fault', self._on_clear_fault)

        self.create_timer(monitor_period, self._monitor)
        self._publish_state()
        self.get_logger().info('Recovery supervisor ready.')

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    def _set_state(self, state, reason):
        with self._state_lock:
            changed = (state != self._state)
            self._state = state
            self._reason = reason
        if changed:
            self.get_logger().info(f'Recovery state -> {self._state_name(state)} ({reason})')
        self._publish_state()

    @staticmethod
    def _state_name(s):
        return {RecoveryStatus.OK: 'OK',
                RecoveryStatus.RECOVERING: 'RECOVERING',
                RecoveryStatus.FAULT: 'FAULT'}.get(s, str(s))

    def _publish_state(self):
        msg = RecoveryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._state_lock:
            msg.state = self._state
            msg.reason = self._reason
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Localization monitoring
    # ------------------------------------------------------------------

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        cov = msg.pose.covariance[0] + msg.pose.covariance[7]  # var(x) + var(y)
        yaw_cov = msg.pose.covariance[35]                      # var(yaw)
        self._latest_cov = cov
        self._latest_yaw_cov = yaw_cov
        if cov <= self._good_cov and yaw_cov <= self._good_yaw_cov:
            self._last_good_pose = msg

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        # A pose was set on /initialpose (manually via RViz, or by us during a
        # re-seed). Adopt it as the reference and pause divergence checks while
        # AMCL converges, so a manual correction is never immediately overridden.
        self._last_good_pose = msg
        self._diverged_since = None
        self._grace_until = time.monotonic() + self._manual_grace
        self.get_logger().info(
            f'/initialpose received; honoring it and pausing divergence checks '
            f'for {self._manual_grace:.0f}s.')

    def _monitor(self) -> None:
        # Only auto-act while OK; never interfere during RECOVERING/FAULT.
        with self._state_lock:
            state = self._state
        if state != RecoveryStatus.OK:
            return
        now = time.monotonic()
        if now < self._grace_until:
            self._diverged_since = None  # within grace after a pose set; let AMCL converge
            return
        cov = self._latest_cov
        yaw_cov = self._latest_yaw_cov
        if cov is None or yaw_cov is None:
            return
        diverged = (cov >= self._diverged_cov) or (yaw_cov >= self._diverged_yaw_cov)
        if diverged:
            if self._diverged_since is None:
                self._diverged_since = now
            elif (now - self._diverged_since) >= self._divergence_time:
                self._diverged_since = None
                self._trigger_relocalization()
        else:
            self._diverged_since = None

    def _trigger_relocalization(self) -> None:
        if self._recovering.is_set():
            return
        if self._last_good_pose is None:
            self.get_logger().error(
                'Localization diverged but no last-known-good pose is available -> FAULT.')
            self._set_state(RecoveryStatus.FAULT, 'localization diverged, no recovery pose')
            return
        self._recovering.set()
        threading.Thread(target=self._relocalize_worker, daemon=True).start()

    def _relocalize_worker(self) -> None:
        try:
            self._set_state(RecoveryStatus.RECOVERING, 'localization diverged; relocalizing')
            # Give consumers a moment to cancel their goals (the contract) before we spin.
            time.sleep(self._settle_time)

            for attempt in range(1, self._max_attempts + 1):
                if not rclpy.ok():
                    return
                self.get_logger().info(
                    f'Relocalization attempt {attempt}/{self._max_attempts}: re-seed + spin.')
                self._reseed_amcl()
                time.sleep(0.5)
                self._spin(self._spin_yaw)
                time.sleep(self._post_spin_settle)
                if (self._latest_cov is not None and self._latest_yaw_cov is not None
                        and self._latest_cov <= self._good_cov
                        and self._latest_yaw_cov <= self._good_yaw_cov):
                    self.get_logger().info('Localization recovered.')
                    self._set_state(RecoveryStatus.OK, 'localization recovered')
                    return

            self.get_logger().error('Relocalization failed after retries -> FAULT.')
            self._set_state(RecoveryStatus.FAULT, 'localization could not be recovered')
        finally:
            self._recovering.clear()

    def _reseed_amcl(self) -> None:
        pose = self._last_good_pose
        if pose is None:
            return
        seed = PoseWithCovarianceStamped()
        seed.header.stamp = self.get_clock().now().to_msg()
        seed.header.frame_id = pose.header.frame_id or 'map'
        seed.pose.pose = pose.pose.pose
        # Moderate covariance so AMCL re-converges from near the last good pose.
        cov = [0.0] * 36
        cov[0] = 0.25     # var(x)
        cov[7] = 0.25     # var(y)
        cov[35] = 0.068   # var(yaw) ~ (15 deg)^2
        seed.pose.covariance = cov
        self._initialpose_pub.publish(seed)
        self.get_logger().info('Re-seeded AMCL to last-known-good pose via /initialpose.')

    def _spin(self, yaw: float) -> None:
        if not self._spin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Spin action server unavailable; skipping spin.')
            return
        goal = Spin.Goal()
        goal.target_yaw = float(yaw)
        done = threading.Event()

        def on_result(_):
            done.set()

        def on_goal(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('Spin goal rejected.')
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        self._spin_client.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=max(15.0, abs(yaw) * 5.0))

    # ------------------------------------------------------------------
    # Fault services
    # ------------------------------------------------------------------

    def _on_report_fault(self, request, response):
        self.get_logger().error('Unrecoverable fault reported by an external node -> FAULT.')
        self._set_state(RecoveryStatus.FAULT, 'fault reported by external node')
        response.success = True
        response.message = 'FAULT latched. Call /clear_fault to resume.'
        return response

    def _on_clear_fault(self, request, response):
        with self._state_lock:
            was_fault = (self._state == RecoveryStatus.FAULT)
        self._set_state(RecoveryStatus.OK, 'fault cleared by operator')
        response.success = True
        response.message = ('Recovery state reset to OK.' if was_fault
                            else 'No fault was active; state is OK.')
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RecoverySupervisor()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
