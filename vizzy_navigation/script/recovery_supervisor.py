#!/usr/bin/env python3
#
# Copyright 2026, João Penha Lopes. All rights reserved.
#
# Recovery supervisor for Vizzy.
#
# A standalone, behavior-agnostic node that:
# monitors localization health (AMCL pose covariance),
# attempts automatic relocalization (re-seed AMCL to the last-known-good
# pose, then sweep: partial spins interleaved with short collision-checked
# forward drives so AMCL gets the motion it needs to re-converge) when
# localization diverges, useful when the map is stale (e.g. tables/chairs
# moved) and AMCL drifts,
# owns a single system recovery state (OK / RECOVERING / FAULT) and
# broadcasts it on /recovery_status so ANY behavior (patrol, etc.) can react,
# accepts unrecoverable-fault reports from other nodes (/report_fault) and
# latches FAULT until a human clears it (/clear_fault).
#
# Contract: while state != OK, consumers MUST stop commanding the base (cancel
# their navigation goals). During RECOVERING the supervisor itself drives the
# base (spin + forward drive) to relocalize; during FAULT the robot stays put.

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import Spin, DriveOnHeading
from builtin_interfaces.msg import Duration
from vizzy_msgs.msg import RecoveryStatus


class RecoverySupervisor(Node):

    def __init__(self):
        super().__init__('recovery_supervisor')

        # --- Parameters ---
        # Detection uses the POSITION covariance (var x + var y) only. The yaw term
        # is disabled by default (thresholds <= 0): yaw covariance is strongly
        # motion-dependent and the relocalization spin itself inflates it (~0.0035),
        # so it cannot tell "wrong" from "just-spun-but-fine" and caused false
        # triggers / false FAULTs. 
        # NOTE: covariance is confidence, not correctness; a confidently-wrong fix has
        # LOW covariance and is NOT detectable here (caught downstream via
        # planning failures / whole-run-missed -> FAULT).
        self.declare_parameter('good_covariance', 0.004)          # pos var x+y: recovered/store-good below this
        self.declare_parameter('good_yaw_covariance', 0.0)        # yaw var: <=0 disables the yaw term
        self.declare_parameter('diverged_covariance', 0.006)      # pos var x+y: divergence above this
        self.declare_parameter('diverged_yaw_covariance', 0.0)    # yaw var: <=0 disables the yaw term
        self.declare_parameter('divergence_time', 4.0)            # s sustained above threshold
        # Honor an externally-set pose on /initialpose (e.g. RViz): adopt it and
        # pause divergence checks for this long so AMCL converges without an auto re-seed.
        self.declare_parameter('manual_pose_grace_period', 2.0)  # s
        self.declare_parameter('max_relocalize_attempts', 5)   # re-seed cycles
        # Each re-seed cycle runs a sweep of (partial spin + short forward drive)
        # repeated sweep_steps times. Translating between spins gives AMCL more
        # motion to re-converge. 
        # The spin is partial (not a full turn) so there is usually free
        # space ahead to drive into; the forward drive uses the native, collision-
        # checked DriveOnHeading behavior, so it self-limits if the way is blocked.
        self.declare_parameter('relocalize_spin_yaw', 0.785)         # rad per spin step (~45 deg)
        self.declare_parameter('relocalize_sweep_steps', 5)         # spin+drive repeats per re-seed
        self.declare_parameter('relocalize_forward_distance', 0.5)  # m per forward drive
        self.declare_parameter('relocalize_forward_speed', 0.15)    # m/s for the forward drive
        self.declare_parameter('settle_time', 1.5)             # s for consumers to stop before moving
        self.declare_parameter('post_spin_settle', 2.0)        # s for AMCL to re-converge after the sweep
        self.declare_parameter('monitor_period', 0.5)          # s

        self._good_cov = self.get_parameter('good_covariance').value
        self._good_yaw_cov = self.get_parameter('good_yaw_covariance').value
        self._diverged_cov = self.get_parameter('diverged_covariance').value
        self._diverged_yaw_cov = self.get_parameter('diverged_yaw_covariance').value
        self._divergence_time = self.get_parameter('divergence_time').value
        self._manual_grace = self.get_parameter('manual_pose_grace_period').value
        self._max_attempts = self.get_parameter('max_relocalize_attempts').value
        self._spin_yaw = self.get_parameter('relocalize_spin_yaw').value
        self._sweep_steps = self.get_parameter('relocalize_sweep_steps').value
        self._forward_dist = self.get_parameter('relocalize_forward_distance').value
        self._forward_speed = self.get_parameter('relocalize_forward_speed').value
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
        self._drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
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
        yaw_ok = (self._good_yaw_cov <= 0.0) or (yaw_cov <= self._good_yaw_cov)
        if cov <= self._good_cov and yaw_ok:
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

    def _cov_report(self) -> str:
        """Detailed covariance-vs-threshold diagnostic for recovery logging.

        Shows the current position/yaw covariance, both thresholds, and how far
        the current value is from each (absolute delta and % of threshold).
        """
        cov = self._latest_cov
        ycov = self._latest_yaw_cov
        if cov is None or ycov is None:
            return 'localization covariance: no /amcl_pose received yet'

        def delta(v, thr):
            if thr <= 0.0:
                return 'disabled'
            return f'{v - thr:+.5f} ({v / thr * 100.0:.0f}% of thr)'

        return (
            'localization covariance: '
            f'pos var(x+y)={cov:.5f} '
            f'[good<= {self._good_cov:.5f} -> {delta(cov, self._good_cov)}; '
            f'diverged>= {self._diverged_cov:.5f} -> {delta(cov, self._diverged_cov)}]; '
            f'yaw var={ycov:.5f} '
            f'[good<= {self._good_yaw_cov:.5f} -> {delta(ycov, self._good_yaw_cov)}; '
            f'diverged>= {self._diverged_yaw_cov:.5f} -> {delta(ycov, self._diverged_yaw_cov)}]'
        )

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
        diverged = (cov >= self._diverged_cov) or \
            (self._diverged_yaw_cov > 0.0 and yaw_cov >= self._diverged_yaw_cov)
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
        self.get_logger().warn(
            f'Localization divergence confirmed (sustained >= {self._divergence_time:.0f}s). '
            f'{self._cov_report()}')
        if self._last_good_pose is None:
            self.get_logger().error(
                'No last-known-good pose available to relocalize from -> FAULT. '
                f'{self._cov_report()}')
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
                    f'Relocalization attempt {attempt}/{self._max_attempts}: re-seed + sweep.')
                self._reseed_amcl()
                time.sleep(0.5)
                # Sweep: alternate a partial spin with a short, collision-checked
                # forward drive. The translation between spins is what helps AMCL
                # re-converge. The spins vary the heading so the forward drives
                # explore feature-rich directions.
                for step in range(1, self._sweep_steps + 1):
                    if not rclpy.ok():
                        return
                    self.get_logger().info(
                        f'  sweep step {step}/{self._sweep_steps}: spin {self._spin_yaw:.2f} rad '
                        f'+ drive {self._forward_dist:.2f} m forward.')
                    self._spin(self._spin_yaw)
                    self._drive_forward(self._forward_dist)
                time.sleep(self._post_spin_settle)
                yaw_ok = (self._good_yaw_cov <= 0.0) or (
                    self._latest_yaw_cov is not None
                    and self._latest_yaw_cov <= self._good_yaw_cov)
                if self._latest_cov is not None and self._latest_cov <= self._good_cov and yaw_ok:
                    self.get_logger().info(f'Localization recovered. {self._cov_report()}')
                    self._set_state(RecoveryStatus.OK, 'localization recovered')
                    return
                self.get_logger().warn(
                    f'Attempt {attempt}/{self._max_attempts} did not recover localization '
                    f'(still above the good thresholds). {self._cov_report()}')

            self.get_logger().error(
                f'Relocalization FAILED after {self._max_attempts} attempt(s) -> FAULT. '
                f'{self._cov_report()}')
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

    def _run_action(self, client, goal, name: str, timeout: float) -> None:
        """Send a behavior_server action goal and block until it finishes.

        Best-effort: an unavailable server or a rejected goal is logged and
        skipped rather than aborting the whole recovery sweep.
        """
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'{name} action server unavailable; skipping.')
            return
        done = threading.Event()

        def on_result(_):
            done.set()

        def on_goal(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn(f'{name} goal rejected.')
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        client.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=timeout)

    def _spin(self, yaw: float) -> None:
        goal = Spin.Goal()
        goal.target_yaw = float(yaw)
        self._run_action(self._spin_client, goal, 'Spin', timeout=max(15.0, abs(yaw) * 5.0))

    def _drive_forward(self, distance: float) -> None:
        # DriveOnHeading drives +x (forward along the current heading). It is
        # collision-checked against the live local costmap, so it stops/aborts if
        # the way is blocked, meaning it is safe even while mislocalized (the obstacle layer
        # comes from the laser, not the static map).
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance)
        goal.speed = float(self._forward_speed)
        secs = int(abs(distance) / max(self._forward_speed, 0.01)) + 5
        goal.time_allowance = Duration(sec=secs)
        self._run_action(self._drive_client, goal, 'DriveOnHeading', timeout=float(secs + 5))

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
