#!/usr/bin/env python3
"""
School traffic control decision node.

Acts as a crossing-guard state machine: the robot base holds a "middle of
the road" pose while no vehicle needs attention. When a vehicle is detected
within a configurable range, the robot holds its position and makes a stop
gesture with the arm. If the vehicle's plate is confirmed as registered
(and, optionally, no pedestrian is in the crossing zone), the robot moves
to a "pass" pose while simultaneously making a pass gesture, letting the
vehicle through. Once the vehicle has left the range, the robot immediately
returns to the middle pose with the default gesture.

A pedestrian within the alert range is handled separately from the state
machine: the robot only speaks a warning through the Say skill and never
changes its base pose or arm gesture because of a pedestrian — traffic
control (the vehicle logic above) remains its only motion-driving task.

Inputs:
  <vehicles_topic>       traffic_perception_msgs/VehicleDetectionArray
  <pedestrians_topic>    traffic_perception_msgs/PedestrianDetectionArray
  <plate_allowed_topic>  std_msgs/Bool  (True = plate is registered/allowed)

Outputs (action clients):
  <go_to_xy_phi_action>   base_navigation/action/GoToXYPhi      (base motion)
  <play_motion2_action>   play_motion2_msgs/action/PlayMotion2  (arm gesture)
  <say_action>            communication_skills/action/Say       (pedestrian alert)

State machine (vehicles only):
  MIDDLE_IDLE   -> base at pose A, default arm gesture. Waiting for a vehicle.
  VEHICLE_STOP  -> base stays at pose A, stop gesture. Waiting for plate OK.
  VEHICLE_PASS  -> base moves to pose B + pass gesture, sent together.
  RETURNING     -> base moves back to pose A + default gesture, sent together.
                   Once both actions complete, state returns to MIDDLE_IDLE.
"""

from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Bool
from traffic_perception_msgs.msg import VehicleDetectionArray, PedestrianDetectionArray

from base_navigation.action import GoToXYPhi
from play_motion2_msgs.action import PlayMotion2
from communication_skills.action import Say


class State(Enum):
    MIDDLE_IDLE = auto()
    VEHICLE_STOP = auto()
    VEHICLE_PASS = auto()
    RETURNING = auto()


class SchoolTrafficControlNode(Node):

    def __init__(self):
        super().__init__('school_traffic_control_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('vehicles_topic', '/perception/vehicles')
        self.declare_parameter('pedestrians_topic', '/perception/pedestrians')
        self.declare_parameter('plate_allowed_topic', '/perception/plate_allowed')

        self.declare_parameter('go_to_xy_phi_action', 'go_to_xy_phi')
        self.declare_parameter('play_motion2_action', 'play_motion2')
        self.declare_parameter('say_action', '/skill/say')

        # Pose A: middle of the road (default holding pose)
        self.declare_parameter('pose_a_x', 0.0)
        self.declare_parameter('pose_a_y', 0.0)
        self.declare_parameter('pose_a_phi_deg', 0.0)

        # Pose B: pulled aside / clear of the lane, used while a vehicle passes
        self.declare_parameter('pose_b_x', 0.0)
        self.declare_parameter('pose_b_y', -3.0)
        self.declare_parameter('pose_b_phi_deg', 0.0)

        # Arm gesture motion names (must be defined in the play_motion2_mgr
        # `motions` parameter — see config/arm_motions.yaml in this package)
        self.declare_parameter('motion_default', 'default_gesture')
        self.declare_parameter('motion_stop', 'stop_gesture')
        self.declare_parameter('motion_pass', 'pass_gesture')

        # Vehicle is "in range" for the crossing decision between these two
        # distances [m] (near, far) — e.g. 2 m to 15 m from the robot.
        self.declare_parameter('range_near_m', 2.0)
        self.declare_parameter('range_far_m', 15.0)

        # Optional safety gate: if true, a pedestrian within pedestrian_zone_m
        # blocks the pass decision even if the plate is allowed. Defaults to
        # false — pedestrians never alter the robot's motion, only trigger
        # the spoken alert below, since traffic control is the robot's only
        # motion-driving task.
        self.declare_parameter('pedestrian_safety_gate', False)
        self.declare_parameter('pedestrian_zone_m', 5.0)

        # Pedestrian alert: within this range, warn verbally via the Say
        # skill — no base or arm motion is triggered by this.
        self.declare_parameter('pedestrian_alert_range_m', 5.0)
        self.declare_parameter('pedestrian_alert_message', 'Caution, pedestrian, please stand clear.')
        self.declare_parameter('pedestrian_alert_cooldown_s', 5.0)

        # Messages older than this are treated as stale/unknown.
        self.declare_parameter('message_timeout_s', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)

        vehicles_topic = self.get_parameter('vehicles_topic').value
        pedestrians_topic = self.get_parameter('pedestrians_topic').value
        plate_allowed_topic = self.get_parameter('plate_allowed_topic').value

        go_to_xy_phi_action = self.get_parameter('go_to_xy_phi_action').value
        play_motion2_action = self.get_parameter('play_motion2_action').value
        say_action = self.get_parameter('say_action').value

        self._pose_a = (
            self.get_parameter('pose_a_x').value,
            self.get_parameter('pose_a_y').value,
            self.get_parameter('pose_a_phi_deg').value,
        )
        self._pose_b = (
            self.get_parameter('pose_b_x').value,
            self.get_parameter('pose_b_y').value,
            self.get_parameter('pose_b_phi_deg').value,
        )

        #self._motion_default = self.get_parameter('motion_default').value
        #self._motion_stop = self.get_parameter('motion_stop').value
        #self._motion_pass = self.get_parameter('motion_pass').value
        self._motion_default = "initial_pose"
        self._motion_stop = "norway_init_stop"
        self._motion_pass = "norway_init_pass"


        self._range_near = float(self.get_parameter('range_near_m').value)
        self._range_far = float(self.get_parameter('range_far_m').value)

        self._pedestrian_safety_gate = bool(self.get_parameter('pedestrian_safety_gate').value)
        self._pedestrian_zone_m = float(self.get_parameter('pedestrian_zone_m').value)

        self._pedestrian_alert_range = float(self.get_parameter('pedestrian_alert_range_m').value)
        self._pedestrian_alert_message = self.get_parameter('pedestrian_alert_message').value
        self._pedestrian_alert_cooldown = Duration(
            seconds=float(self.get_parameter('pedestrian_alert_cooldown_s').value))

        self._msg_timeout = Duration(seconds=float(self.get_parameter('message_timeout_s').value))
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        # ── Sensor state ─────────────────────────────────────────────────
        self._vehicles = None
        self._vehicles_stamp = None
        self._pedestrians = None
        self._pedestrians_stamp = None
        self._plate_allowed = False
        self._plate_stamp = None

        self.create_subscription(VehicleDetectionArray, vehicles_topic, self._vehicles_cb, 10)
        self.create_subscription(PedestrianDetectionArray, pedestrians_topic, self._pedestrians_cb, 10)
        self.create_subscription(Bool, plate_allowed_topic, self._plate_cb, 10)

        # ── Action clients ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, GoToXYPhi, go_to_xy_phi_action)
        self._motion_client = ActionClient(self, PlayMotion2, play_motion2_action)
        self._say_client = ActionClient(self, Say, say_action)

        self._nav_goal_handle = None
        self._motion_goal_handle = None

        # Used only while in RETURNING, to know when both actions finished.
        self._nav_done = True
        self._motion_done = True

        # Pedestrian alert bookkeeping (independent of the state machine).
        self._say_in_flight = False
        self._last_say_stamp = None

        # ── State machine ────────────────────────────────────────────────
        self._state = State.MIDDLE_IDLE
        self._send_motion(self._motion_default)

        self._timer = self.create_timer(1.0 / control_rate_hz, self._tick)

        self.get_logger().info('school_traffic_control_node ready — state=MIDDLE_IDLE')

    # ── Subscription callbacks ──────────────────────────────────────────

    def _vehicles_cb(self, msg: VehicleDetectionArray):
        self._vehicles = msg.vehicles
        self._vehicles_stamp = self.get_clock().now()

    def _pedestrians_cb(self, msg: PedestrianDetectionArray):
        self._pedestrians = msg.pedestrians
        self._pedestrians_stamp = self.get_clock().now()

    def _plate_cb(self, msg: Bool):
        self._plate_allowed = msg.data
        self._plate_stamp = self.get_clock().now()

    # ── Freshness / sensor helpers ──────────────────────────────────────

    def _is_fresh(self, stamp: Time) -> bool:
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) < self._msg_timeout

    def _closest_vehicle_in_range(self):
        if self._vehicles is None or not self._is_fresh(self._vehicles_stamp):
            return None
        candidates = [v for v in self._vehicles if self._range_near <= v.distance <= self._range_far]
        if not candidates:
            return None
        return min(candidates, key=lambda v: v.distance)

    def _pedestrian_blocking(self) -> bool:
        if not self._pedestrian_safety_gate:
            return False
        if self._pedestrians is None or not self._is_fresh(self._pedestrians_stamp):
            return False
        return any(p.distance <= self._pedestrian_zone_m for p in self._pedestrians)

    def _plate_ok(self) -> bool:
        return self._plate_allowed and self._is_fresh(self._plate_stamp)

    def _pedestrian_in_alert_range(self) -> bool:
        if self._pedestrians is None or not self._is_fresh(self._pedestrians_stamp):
            return False
        return any(p.distance <= self._pedestrian_alert_range for p in self._pedestrians)

    # ── Control loop / state machine ────────────────────────────────────

    def _tick(self):
        self._maybe_alert_pedestrian()

        target_vehicle = self._closest_vehicle_in_range()

        if self._state == State.MIDDLE_IDLE:
            if target_vehicle is not None:
                self._enter_vehicle_stop()

        elif self._state == State.VEHICLE_STOP:
            if target_vehicle is None:
                # Vehicle left the range without being cleared to pass.
                self._enter_idle()
            elif self._plate_ok() and not self._pedestrian_blocking():
                self._enter_vehicle_pass()

        elif self._state == State.VEHICLE_PASS:
            if target_vehicle is None:
                # Vehicle has passed through — head back immediately.
                self._enter_returning()

        elif self._state == State.RETURNING:
            if self._nav_done and self._motion_done:
                self._state = State.MIDDLE_IDLE
                self.get_logger().info('Back at middle pose — state=MIDDLE_IDLE')

    def _maybe_alert_pedestrian(self):
        """Speak a warning when a pedestrian is close. Never touches motion."""
        if not self._pedestrian_in_alert_range() or self._say_in_flight:
            return
        if self._last_say_stamp is not None and (self.get_clock().now() - self._last_say_stamp) < \
                self._pedestrian_alert_cooldown:
            return
        self._send_say(self._pedestrian_alert_message)

    # ── State transitions ───────────────────────────────────────────────

    def _enter_vehicle_stop(self):
        self._state = State.VEHICLE_STOP
        self.get_logger().info('Vehicle in range — state=VEHICLE_STOP (stop gesture)')
        self._send_motion(self._motion_stop)

    def _enter_vehicle_pass(self):
        self._state = State.VEHICLE_PASS
        self.get_logger().info('Plate allowed — state=VEHICLE_PASS (move to pose B + pass gesture)')
        #self._send_nav_goal(self._pose_b)
        self._send_motion(self._motion_pass)

    def _enter_returning(self):
        self._state = State.RETURNING
        self._nav_done = False
        self._motion_done = False
        self.get_logger().info('Vehicle passed — state=RETURNING (move to pose A + default gesture)')
        #self._send_nav_goal(self._pose_a)
        self._send_motion(self._motion_default)

    def _enter_idle(self):
        self._state = State.MIDDLE_IDLE
        self.get_logger().info('Vehicle left range — state=MIDDLE_IDLE (default gesture)')
        self._send_motion(self._motion_default)

    # ── Action helpers ───────────────────────────────────────────────────

    def _send_nav_goal(self, pose):
        x, y, phi = pose
        goal = GoToXYPhi.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.phi = float(phi)

        self._cancel_goal(self._nav_goal_handle)

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('go_to_xy_phi action server not available')
            self._nav_done = True
            return

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response_cb)

    def _send_motion(self, motion_name: str):
        goal = PlayMotion2.Goal()
        goal.motion_name = motion_name
        goal.skip_planning = False

        self._cancel_goal(self._motion_goal_handle)

        if not self._motion_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('play_motion2 action server not available')
            self._motion_done = True
            return

        future = self._motion_client.send_goal_async(goal)
        future.add_done_callback(self._motion_goal_response_cb)

    def _send_say(self, text: str):
        goal = Say.Goal()
        goal.meta.caller = self.get_name()
        goal.meta.priority = goal.meta.NORMAL_PRIORITY
        goal.person_id = ''
        goal.group_id = ''
        goal.input = text

        if not self._say_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Say action server not available')
            return

        self._say_in_flight = True
        future = self._say_client.send_goal_async(goal)
        future.add_done_callback(self._say_goal_response_cb)

    def _say_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Say goal rejected')
            self._say_in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._say_result_cb)

    def _say_result_cb(self, future):
        result = future.result().result.result
        self.get_logger().info(f'Say result: error_code={result.error_code} ({result.error_msg})')
        self._say_in_flight = False
        self._last_say_stamp = self.get_clock().now()

    @staticmethod
    def _cancel_goal(goal_handle):
        if goal_handle is not None and goal_handle.status in (1, 2):  # ACCEPTED, EXECUTING
            goal_handle.cancel_goal_async()

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('go_to_xy_phi goal rejected')
            self._nav_done = True
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _motion_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('play_motion2 goal rejected')
            self._motion_done = True
            return
        self._motion_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._motion_result_cb)

    def _nav_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'go_to_xy_phi result: success={result.success} ({result.message})')
        self._nav_done = True

    def _motion_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'play_motion2 result: success={result.success} ({result.error})')
        self._motion_done = True


def main(args=None):
    rclpy.init(args=args)
    node = SchoolTrafficControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
