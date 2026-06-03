#!/usr/bin/env python3
#
# Copyright 2026, João Penha Lopes. All rights reserved.
#
# Autonomous patrol node for Vizzy.
#
# Loops through a list of waypoints loaded from a YAML file using Nav2's
# FollowWaypoints action. Monitors battery state between and during patrol runs.
# When LOW_BATTERY is detected, Vizzy cancels the active navigation goal, executes the
# existing charging procedure via the charge action server, waits for battery
# recovery, undocks, then immediately resumes patrol from waypoint 0.
#
# Battery policy:
#   - Monitors the aggregated /battery_status topic (Kokam + both Segway
#     batteries). Docks if ANY source is low (any_low); leaves the dock only once
#     ALL sources are full (all_full), since they share one charge connector.
#   - Real hardware: /battery_status MUST be published. The node blocks at the
#     first check until a message arrives.
#   - Simulation (is_simulation=True): if no /battery_status is available, a
#     warning is logged and patrol continues uninterrupted.

import threading
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from std_srvs.srv import Trigger
from vizzy_msgs.action import Charge
from vizzy_msgs.msg import BatteryStatus


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        # Declare and get parameters.
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('is_simulation', False)
        self.declare_parameter('battery_check_period_s', 5.0)
        waypoints_file             = self.get_parameter('waypoints_file').value
        self._is_simulation        = self.get_parameter('is_simulation').value
        self._battery_check_period = self.get_parameter('battery_check_period_s').value

        self.get_logger().info(
            f'Starting patrol_node '
            f'[is_simulation={self._is_simulation}, '
            f'battery_check_period={self._battery_check_period}s, '
            f'waypoints_file={waypoints_file}]')

        self._waypoints = self._load_waypoints(waypoints_file)

        self._follow_wp_client    = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._charge_client       = ActionClient(self, Charge,           'charge')
        self._nav2_active_client  = self.create_client(
            Trigger, '/lifecycle_manager_navigation/is_active')

        # Aggregated battery status from the battery monitor (Kokam + both Segway
        # batteries). Cached from the topic; the patrol logic reads the latest.
        self._battery_status = None
        self._battery_lock   = threading.Lock()
        self.create_subscription(
            BatteryStatus, 'battery_status', self._on_battery_status, 10)

        # Protected by _goal_lock. Holds the currently active action goal handle
        # so it can be cancelled from the battery-monitor path.
        self._active_goal_handle = None
        self._goal_lock          = threading.Lock()

        # Manual docking control. _dock_requested asks the patrol loop to dock
        # and hold (regardless of battery); _resume_requested releases the hold.
        # See the dock_now / resume_patrol services below.
        self._dock_requested   = threading.Event()
        self._resume_requested = threading.Event()
        self.create_service(Trigger, 'dock_now',      self._on_dock_now)
        self.create_service(Trigger, 'resume_patrol', self._on_resume_patrol)

        self._patrol_thread = threading.Thread(target=self._patrol_loop, daemon=True)
        self._patrol_thread.start()

    # ------------------------------------------------------------------
    # Waypoint loading
    # ------------------------------------------------------------------

    def _load_waypoints(self, path: str) -> list:
        if not path:
            raise RuntimeError('waypoints_file parameter is empty. Cannot start patrol.')
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            poses = []
            for wp in data['waypoints']:
                pose = PoseStamped()
                pose.header.frame_id   = wp['frame_id']
                pose.pose.position.x   = float(wp['position']['x'])
                pose.pose.position.y   = float(wp['position']['y'])
                pose.pose.position.z   = float(wp['position']['z'])
                pose.pose.orientation.x = float(wp['orientation']['x'])
                pose.pose.orientation.y = float(wp['orientation']['y'])
                pose.pose.orientation.z = float(wp['orientation']['z'])
                pose.pose.orientation.w = float(wp['orientation']['w'])
                poses.append(pose)
            if not poses:
                raise RuntimeError('waypoints list is empty in YAML file.')
            self.get_logger().info(f'Loaded {len(poses)} waypoints from {path}')
            return poses
        except Exception as e:
            self.get_logger().fatal(f'Failed to load waypoints from "{path}": {e}')
            raise

    # ------------------------------------------------------------------
    # Nav2 readiness
    # ------------------------------------------------------------------

    def _wait_for_nav2_active(self) -> None:
        """Block until the Nav2 lifecycle manager reports all nodes as active."""
        self.get_logger().info('Waiting for Nav2 stack to become active...')
        while rclpy.ok():
            if not self._nav2_active_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info(
                    'lifecycle_manager_navigation/is_active not yet reachable, retrying...',
                    throttle_duration_sec=10.0)
                continue

            future = self._nav2_active_client.call_async(Trigger.Request())
            deadline = time.monotonic() + 3.0
            while not future.done() and rclpy.ok() and time.monotonic() < deadline:
                time.sleep(0.05)

            if future.done() and future.result() is not None and future.result().success:
                self.get_logger().info('Nav2 stack is active. Starting patrol.')
                return

            self.get_logger().info(
                'Nav2 not yet active, retrying in 1 s...',
                throttle_duration_sec=5.0)
            time.sleep(1.0)

    # ------------------------------------------------------------------
    # Battery monitoring
    # ------------------------------------------------------------------

    def _on_battery_status(self, msg: BatteryStatus) -> None:
        with self._battery_lock:
            self._battery_status = msg

    def _latest_battery_status(self):
        """Return the latest BatteryStatus.

        Real hardware: if none has been received yet, block until one arrives
        (the battery monitor must be running). Simulation: return None so patrol
        continues without a battery check.
        """
        with self._battery_lock:
            msg = self._battery_status
        if msg is not None:
            return msg
        if self._is_simulation:
            self.get_logger().warn(
                'No battery_status received in simulation. '
                'Continuing patrol without battery check.',
                throttle_duration_sec=60.0)
            return None
        self.get_logger().error(
            'No battery_status received on real hardware. '
            'Blocking patrol until it appears...')
        while rclpy.ok():
            time.sleep(1.0)
            with self._battery_lock:
                msg = self._battery_status
            if msg is not None:
                return msg
            self.get_logger().error(
                'Still waiting for battery_status...', throttle_duration_sec=10.0)
        return None

    def _battery_is_ok(self) -> bool:
        """Return False if ANY battery source is low (Kokam or either Segway battery)."""
        msg = self._latest_battery_status()
        if msg is None:
            return True  # simulation fallback or node shutting down
        if msg.any_low:
            detail = ', '.join(msg.alerts) if msg.alerts else 'a source is low'
            self.get_logger().warn(f'LOW BATTERY: {detail}. Interrupting patrol.')
            return False
        return True

    def _wait_for_battery_ok(self) -> None:
        """Block until ALL battery sources are full.

        All sources charge through one connector, so we leave the dock only when
        the last source finishes (all_full), not just the Kokam.
        Simulation: wait a fixed 15 s to represent a charge event.
        """
        if self._is_simulation:
            self.get_logger().info(
                'Simulation: waiting 15 s to simulate charging time.')
            time.sleep(15.0)
            return

        self.get_logger().info('Waiting for all battery sources to reach full...')
        while rclpy.ok():
            with self._battery_lock:
                msg = self._battery_status
            if msg is not None and msg.all_full:
                self.get_logger().info('All batteries full. Ready to resume patrol.')
                return
            time.sleep(5.0)

    # ------------------------------------------------------------------
    # Generic action helper
    # ------------------------------------------------------------------

    def _send_action_sync(self, client: ActionClient, goal, description: str):
        """Send an action goal and block until result arrives.

        Returns (GoalStatus code, result object) or (None, None) on failure.
        The active goal handle is stored in self._active_goal_handle while the
        goal is running so it can be cancelled externally.
        """
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'{description}: action server not available.')
            return None, None

        done_event    = threading.Event()
        result_holder = [None]

        def on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(f'{description}: goal rejected.')
                done_event.set()
                return
            with self._goal_lock:
                self._active_goal_handle = handle
            handle.get_result_async().add_done_callback(on_result)

        def on_result(future):
            result_holder[0] = future.result()
            with self._goal_lock:
                self._active_goal_handle = None
            done_event.set()

        client.send_goal_async(goal).add_done_callback(on_goal_response)
        done_event.wait()

        r = result_holder[0]
        return (r.status if r else None), (r.result if r else None)

    def _cancel_active_goal(self) -> None:
        with self._goal_lock:
            handle = self._active_goal_handle
        if handle is not None:
            self.get_logger().info('Cancelling active navigation goal.')
            handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # Navigation and charging
    # ------------------------------------------------------------------

    def _follow_waypoints(self) -> bool:
        """Send all patrol waypoints to Nav2's FollowWaypoints action server."""
        now = self.get_clock().now().to_msg()
        stamped = []
        for wp in self._waypoints:
            p = PoseStamped()
            p.header.frame_id = wp.header.frame_id
            p.header.stamp    = now
            p.pose            = wp.pose
            stamped.append(p)

        goal       = FollowWaypoints.Goal()
        goal.poses = stamped

        status, _ = self._send_action_sync(
            self._follow_wp_client, goal, 'FollowWaypoints')
        return status == GoalStatus.STATUS_SUCCEEDED

    def _dock(self) -> bool:
        goal      = Charge.Goal()
        goal.goal = Charge.Goal.CHARGE
        status, result = self._send_action_sync(self._charge_client, goal, 'Charge(CHARGE)')
        if status == GoalStatus.STATUS_SUCCEEDED and result is not None:
            return result.result == Charge.Result.CHARGE_SUCCESS
        return False

    def _undock(self) -> bool:
        goal      = Charge.Goal()
        goal.goal = Charge.Goal.STOP_CHARGE
        status, result = self._send_action_sync(self._charge_client, goal, 'Charge(STOP_CHARGE)')
        if status == GoalStatus.STATUS_SUCCEEDED and result is not None:
            return result.result == Charge.Result.STOPPED
        return False

    def _charge_cycle(self) -> None:
        self.get_logger().info('--- Charge cycle start ---')

        if not self._dock():
            self.get_logger().error(
                'Docking failed. Skipping charge cycle and resuming patrol.')
            return

        self.get_logger().info('Docked. Waiting for battery to recover...')
        self._wait_for_battery_ok()

        self.get_logger().info('Undocking...')
        if not self._undock():
            self.get_logger().error(
                'Undocking failed. Manual intervention may be required.')

        self.get_logger().info('--- Charge cycle end ---')

    # ------------------------------------------------------------------
    # Manual docking control
    # ------------------------------------------------------------------

    def _on_dock_now(self, request, response):
        """Trigger service: dock now and hold, regardless of battery state.

        Sets a flag the patrol loop acts on, and cancels any active waypoint
        goal immediately so the robot stops patrolling without waiting for the
        next battery-poll tick. The robot then docks and holds at the dock
        until resume_patrol is called.
        """
        self.get_logger().info('Manual dock requested via dock_now service.')
        self._dock_requested.set()
        self._cancel_active_goal()
        response.success = True
        response.message = ('Dock request accepted. Robot will dock and hold '
                            'until /resume_patrol is called.')
        return response

    def _on_resume_patrol(self, request, response):
        """Trigger service: release a held dock and resume patrol (undocks)."""
        self.get_logger().info('Resume requested via resume_patrol service.')
        self._resume_requested.set()
        response.success = True
        response.message = ('Resume request accepted. Robot will undock (if '
                            'holding at the dock) and resume patrol.')
        return response

    def _manual_dock_cycle(self) -> None:
        """Dock on manual request and hold until resume_patrol is called.

        Unlike _charge_cycle (battery-driven), this does not wait for battery
        recovery and does not auto-undock; it parks at the dock until released.
        """
        self._dock_requested.clear()
        self.get_logger().info('--- Manual dock cycle start (dock and hold) ---')

        # Run the docking maneuver, then hold regardless of whether charging is
        # confirmed. A manual dock may be triggered with a near-full battery (no
        # measurable charge slope) or just to test the maneuver, so we do not
        # resume patrol on a "not charging" result the way the battery-driven
        # cycle does. The robot stays parked until /resume_patrol is called.
        if self._dock():
            self.get_logger().info('Docked and charging confirmed.')
        else:
            self.get_logger().warn(
                'Dock did not confirm charging (near-full battery or no contact). '
                'Holding at the dock anyway. If the robot is NOT physically docked, '
                'call /resume_patrol to release it.')

        # Discard any resume request that arrived before we finished docking.
        self._resume_requested.clear()
        self.get_logger().info('Holding at dock until /resume_patrol is called.')

        while rclpy.ok() and not self._resume_requested.is_set():
            time.sleep(0.5)
        if not rclpy.ok():
            return
        self._resume_requested.clear()

        self.get_logger().info('Resuming: undocking...')
        if not self._undock():
            self.get_logger().error(
                'Undocking failed. Manual intervention may be required.')

        self.get_logger().info('--- Manual dock cycle end ---')

    # ------------------------------------------------------------------
    # Main patrol loop
    # ------------------------------------------------------------------

    def _patrol_loop(self) -> None:
        self.get_logger().info('Patrol loop started.')

        # Wait for Nav2 to be active before starting patrol runs.
        self._wait_for_nav2_active()

        while rclpy.ok():
            # Pre-flight checks before starting a new patrol run. A manual dock
            # request takes priority over the battery check.
            if self._dock_requested.is_set():
                self._manual_dock_cycle()
                continue
            if not self._battery_is_ok():
                self._charge_cycle()
                continue

            self.get_logger().info('Starting patrol run...')

            patrol_done      = threading.Event()
            patrol_succeeded = [False]

            def _run() -> None:
                patrol_succeeded[0] = self._follow_waypoints()
                patrol_done.set()

            threading.Thread(target=_run, daemon=True).start()

            # Poll battery and the manual-dock flag concurrently while
            # FollowWaypoints is executing.
            battery_low = False
            while not patrol_done.wait(timeout=self._battery_check_period):
                if not rclpy.ok():
                    break
                if self._dock_requested.is_set():
                    # dock_now already cancelled the goal; just stop polling.
                    self._cancel_active_goal()
                    break
                if not self._battery_is_ok():
                    battery_low = True
                    self._cancel_active_goal()
                    break

            patrol_done.wait()  # ensure the action thread exits cleanly.

            # A manual dock request wins over the battery branch. It is the
            # source of truth (set by the dock_now service), so check the flag
            # directly rather than relying on the poll loop having caught it
            # before FollowWaypoints finished cancelling.
            if self._dock_requested.is_set():
                self.get_logger().info(
                    'Patrol interrupted: manual dock request. Docking and holding.')
                self._manual_dock_cycle()
            elif battery_low:
                self.get_logger().info(
                    'Patrol interrupted: low battery. Starting charge cycle.')
                self._charge_cycle()
            elif patrol_succeeded[0]:
                self.get_logger().info(
                    'Patrol run complete. Immediately restarting from waypoint 0.')
            else:
                self.get_logger().warn(
                    'FollowWaypoints returned non-success status. Retrying in 5 s.')
                time.sleep(5.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node     = PatrolNode()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[patrol_node] Fatal error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
