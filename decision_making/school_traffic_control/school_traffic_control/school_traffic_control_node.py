#!/usr/bin/env python3
"""
School traffic control decision node.

Acts as a crossing-guard state machine: the robot base holds a "middle of
the road" pose while no vehicle needs attention. When a vehicle enters the
close range (range_near_m..range_far_m, default 3-5 m), the robot holds
position and makes a stop gesture while watching for the vehicle to
actually come to a stop (VehicleDetection.stopped, from
traffic_object_detection). Once stopped, the robot looks down at the plate
(head motion) and checks it; if confirmed registered within
plate_confirmation_timeout_s, the robot crosses to pose B, waves the
vehicle through, and returns. If not confirmed in time, the robot goes
back to just holding the stop gesture until the vehicle leaves.

A pedestrian within the alert range is handled separately from the state
machine: the robot only speaks a warning through TTS and never changes its
base pose or arm gesture because of a pedestrian — traffic control (the
vehicle logic above) remains its only motion-driving task.

Perception load switching: traffic_object_detection and
vehicle_plate_detection are both heavy YOLO models the Jetson can't
comfortably run at once (see _set_perception_mode). Traffic detection runs
whenever this node needs to see the vehicle itself (MIDDLE_IDLE,
VEHICLE_STOP, WAIT_TO_LEAVE, CROSS_VEHICLE, CHECK_VEHICLE_IN_RANGE,
RETURNING); plate detection only runs during CHECK_PLATE.

Inputs:
  <vehicles_topic>       traffic_perception_msgs/VehicleDetectionArray
  <pedestrians_topic>    traffic_perception_msgs/PedestrianDetectionArray
  <plate_allowed_topic>  std_msgs/Bool  (True = plate is registered/allowed;
                          noise-filtered via a sliding-window vote, see
                          _plate_ok — a single stray misread doesn't flip
                          the decision either way)

Outputs (action clients):
  <go_to_xy_phi_action>   base_navigation/action/GoToXYPhi      (base motion)
  <play_motion2_action>   play_motion2_msgs/action/PlayMotion2  (arm + head gesture)
  <tts_action>             tts_msgs/action/TTS                  (voice announcements)

State machine (vehicles only):
  MIDDLE_IDLE             -> base at pose A, default arm gesture. Waiting for a
                             vehicle in the close range.
  VEHICLE_STOP            -> base stays at pose A, stop gesture. Traffic detection
                             stays on here (need VehicleDetection.stopped) —
                             waiting for the vehicle to actually come to a stop.
                             -> MIDDLE_IDLE if it leaves the range first.
  CHECK_PLATE             -> head motion looks down at the plate; plate detection
                             on, traffic detection off. plate_confirmation_timeout_s
                             timer running.
                             -> CROSS_VEHICLE if the plate is confirmed in time.
                             -> WAIT_TO_LEAVE if the timer expires first.
  WAIT_TO_LEAVE            -> same pose/gesture as VEHICLE_STOP (holding position);
                             plate detection off, traffic detection back on.
                             -> MIDDLE_IDLE once the vehicle leaves the range.
  CROSS_VEHICLE            -> base moves to pose B + pass gesture, sent together.
                             -> CHECK_VEHICLE_IN_RANGE once the base arrives at pose B.
  CHECK_VEHICLE_IN_RANGE   -> pass gesture, then waves in a loop while the vehicle
                             is still in range.
                             -> RETURNING once it leaves.
  RETURNING                -> base moves back to pose A + default gesture, sent
                             together. Once both complete, state returns to
                             MIDDLE_IDLE.
"""

from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool
from traffic_perception_msgs.msg import VehicleDetectionArray, PedestrianDetectionArray

from base_navigation.action import GoToXYPhi
from play_motion2_msgs.action import PlayMotion2
from tts_msgs.action import TTS


class State(Enum):
    MIDDLE_IDLE = auto()
    VEHICLE_STOP = auto()
    CHECK_PLATE = auto()
    WAIT_TO_LEAVE = auto()
    CROSS_VEHICLE = auto()
    CHECK_VEHICLE_IN_RANGE = auto()
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
        self.declare_parameter('tts_action', '/tts_engine/tts')

        # Perception load switching: traffic_object_detection and
        # vehicle_plate_detection are both heavy YOLO models the Jetson
        # can't comfortably run at once, so only one runs at a time —
        # traffic detection by default (including VEHICLE_STOP, which needs
        # it to know when the vehicle actually stops), switched to plate
        # detection only during CHECK_PLATE, then switched back.
        self.declare_parameter('traffic_detection_enabled_topic', '/perception/traffic_object_detection_enabled')
        self.declare_parameter('plate_detection_enabled_topic', '/perception/plate_detection_enabled')

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

        # Vehicle is "in range" (close range) for the whole crossing
        # interaction between these two distances [m] (near, far).
        self.declare_parameter('range_near_m', 3.0)
        self.declare_parameter('range_far_m', 5.0)

        # CHECK_PLATE: give up waiting for a registered-plate confirmation
        # after this long and fall back to WAIT_TO_LEAVE instead.
        self.declare_parameter('plate_confirmation_timeout_s', 15.0)

        # Plate noise filtering: plate_allowed is noise-filtered over a
        # sliding window rather than trusting the single latest reading — a
        # plate counts as confirmed once at least plate_vote_min_yes of the
        # last plate_vote_window readings were True. See _plate_ok.
        self.declare_parameter('plate_vote_window', 5)
        self.declare_parameter('plate_vote_min_yes', 2)

        # Optional safety gate: if true, a pedestrian within pedestrian_zone_m
        # blocks the pass decision even if the plate is allowed. Defaults to
        # false — pedestrians never alter the robot's motion, only trigger
        # the spoken alert below, since traffic control is the robot's only
        # motion-driving task.
        self.declare_parameter('pedestrian_safety_gate', False)
        self.declare_parameter('pedestrian_zone_m', 5.0)

        # Pedestrian alert: within this range, warn verbally via TTS — no
        # base or arm motion is triggered by this.
        self.declare_parameter('pedestrian_alert_range_m', 5.0)
        self.declare_parameter('pedestrian_alert_message', 'Caution, pedestrian, please stand clear.')
        self.declare_parameter('pedestrian_alert_cooldown_s', 5.0)

        # Voice announcements on vehicle state transitions.
        self.declare_parameter('vehicle_stop_message', 'Stop, please. Let the children cross.')
        self.declare_parameter('vehicle_pass_message', 'You may proceed.')

        # Messages older than this are treated as stale/unknown.
        self.declare_parameter('message_timeout_s', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)

        vehicles_topic = self.get_parameter('vehicles_topic').value
        pedestrians_topic = self.get_parameter('pedestrians_topic').value
        plate_allowed_topic = self.get_parameter('plate_allowed_topic').value
        traffic_detection_enabled_topic = self.get_parameter('traffic_detection_enabled_topic').value
        plate_detection_enabled_topic = self.get_parameter('plate_detection_enabled_topic').value

        go_to_xy_phi_action = self.get_parameter('go_to_xy_phi_action').value
        play_motion2_action = self.get_parameter('play_motion2_action').value
        tts_action = self.get_parameter('tts_action').value

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
        self._motion_stop_init = "norway_stop_init"
        self._motion_init_stop = "norway_init_stop"
        self._motion_init_pass = "norway_init_pass"
        self._motion_pass_wave = "norway_pass_wave"
        self._motion_pass_init = "norway_pass_init"
        self._motion_check_plate = "norway_check_plate"


        self._range_near = float(self.get_parameter('range_near_m').value)
        self._range_far = float(self.get_parameter('range_far_m').value)
        self._plate_confirmation_timeout = Duration(
            seconds=float(self.get_parameter('plate_confirmation_timeout_s').value))
        self._plate_vote_min_yes = int(self.get_parameter('plate_vote_min_yes').value)
        plate_vote_window = int(self.get_parameter('plate_vote_window').value)

        self._pedestrian_safety_gate = bool(self.get_parameter('pedestrian_safety_gate').value)
        self._pedestrian_zone_m = float(self.get_parameter('pedestrian_zone_m').value)

        self._pedestrian_alert_range = float(self.get_parameter('pedestrian_alert_range_m').value)
        self._pedestrian_alert_message = self.get_parameter('pedestrian_alert_message').value
        self._pedestrian_alert_cooldown = Duration(
            seconds=float(self.get_parameter('pedestrian_alert_cooldown_s').value))

        self._vehicle_stop_message = self.get_parameter('vehicle_stop_message').value
        self._vehicle_pass_message = self.get_parameter('vehicle_pass_message').value

        self._msg_timeout = Duration(seconds=float(self.get_parameter('message_timeout_s').value))
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        # ── Sensor state ─────────────────────────────────────────────────
        self._vehicles = None
        self._vehicles_stamp = None
        self._pedestrians = None
        self._pedestrians_stamp = None
        self._plate_allowed = False
        self._plate_stamp = None
        # Sliding-window vote of recent plate_allowed readings — see
        # _plate_ok. Cleared on every CHECK_PLATE entry so a previous
        # vehicle's votes can't leak into the next one's decision.
        self._plate_votes = deque(maxlen=plate_vote_window)

        self.create_subscription(VehicleDetectionArray, vehicles_topic, self._vehicles_cb, 10)
        self.create_subscription(PedestrianDetectionArray, pedestrians_topic, self._pedestrians_cb, 10)
        self.create_subscription(Bool, plate_allowed_topic, self._plate_cb, 10)

        # ── Perception load switching ────────────────────────────────────
        # Transient-local so a perception node that (re)starts after a mode
        # switch already happened still gets the current on/off state
        # immediately, instead of waiting for the next state transition.
        perception_mode_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_traffic_enabled = self.create_publisher(
            Bool, traffic_detection_enabled_topic, perception_mode_qos)
        self._pub_plate_enabled = self.create_publisher(
            Bool, plate_detection_enabled_topic, perception_mode_qos)

        # ── Action clients ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, GoToXYPhi, go_to_xy_phi_action)
        self._motion_client = ActionClient(self, PlayMotion2, play_motion2_action)
        self._say_client = ActionClient(self, TTS, tts_action)

        self._nav_goal_handle = None
        self._motion_goal_handle = None

        # Bumped on every _send_nav_goal/_send_motion call so a goal
        # acceptance callback can tell whether it's still the latest
        # request and discard itself otherwise — guards against a goal
        # that becomes current only after being superseded.
        self._nav_request_id = 0
        self._motion_request_id = 0

        # True whenever no nav/motion goal is currently in flight — set
        # False the moment a goal is dispatched, True once its actual
        # terminal result (success, rejection, or cancellation) arrives.
        self._nav_done = True
        self._motion_done = True

        # (request_id, goal_args) queued while a goal is still in flight;
        # dispatched once that goal's real result arrives — sending a
        # replacement goal any earlier gets rejected by the action server,
        # since it treats the previous goal as still busy until then.
        self._nav_pending = None
        self._motion_pending = None

        # Used only while in CHECK_VEHICLE_IN_RANGE, to sequence the pass
        # gesture before waving instead of sending both at once.
        self._pass_gesture_sent = False

        # Set on entering CHECK_PLATE; used for the plate_confirmation_timeout_s
        # fallback to WAIT_TO_LEAVE.
        self._check_plate_entered_at = None

        # Pedestrian alert bookkeeping (independent of the state machine).
        self._say_in_flight = False
        self._last_say_stamp = None

        # ── State machine ────────────────────────────────────────────────
        self._state = State.MIDDLE_IDLE
        self._send_motion(self._motion_default)
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)

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
        self._plate_votes.append(bool(msg.data))

    # ── Perception load switching ────────────────────────────────────────

    def _set_perception_mode(self, traffic_enabled: bool, plate_enabled: bool):
        """Toggle which heavy perception model is allowed to run — the
        Jetson can't comfortably run traffic_object_detection and
        vehicle_plate_detection at once."""
        self._pub_traffic_enabled.publish(Bool(data=traffic_enabled))
        self._pub_plate_enabled.publish(Bool(data=plate_enabled))

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
        """True once at least plate_vote_min_yes of the last
        plate_vote_window plate_allowed readings were True — smooths over
        single-frame OCR noise (e.g. one stray False among mostly-True
        readings) instead of trusting only the single latest message."""
        if not self._is_fresh(self._plate_stamp):
            return False
        return sum(self._plate_votes) >= self._plate_vote_min_yes

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
            # Traffic detection stays on for this whole state (unlike
            # CHECK_PLATE), so target_vehicle is always live here — no
            # staleness caveat needed.
            if target_vehicle is None:
                self._enter_idle()
            elif target_vehicle.stopped:
                self._enter_check_plate()

        elif self._state == State.CHECK_PLATE:
            if self._plate_ok():
                self._enter_cross_vehicle()
            elif (self.get_clock().now() - self._check_plate_entered_at) > self._plate_confirmation_timeout:
                self._enter_wait_to_leave()

        elif self._state == State.WAIT_TO_LEAVE:
            if target_vehicle is None:
                self._enter_idle()

        elif self._state == State.CROSS_VEHICLE:
            if self._nav_done:
                self._enter_check_vehicle_in_range()

        elif self._state == State.CHECK_VEHICLE_IN_RANGE:
            if target_vehicle is None:
                # Vehicle has passed through — head back immediately.
                self._enter_returning()
            elif not self._motion_done:
                # Still finishing the previous motion (the pass gesture or
                # a wave cycle) — wait for it before sending the next one
                # instead of piling goals on top of it.
                pass
            elif not self._pass_gesture_sent:
                self._pass_gesture_sent = True
                self._send_motion(self._motion_init_pass)
            else:
                # Keep waving while the vehicle is in range: only send a
                # new wave once the previous one has finished, not on
                # every 10 Hz tick.
                self._passing_vehicle()

        elif self._state == State.RETURNING:
            if self._nav_done and self._motion_done:
                self._state = State.MIDDLE_IDLE
                self.get_logger().info('Back at middle pose — state=MIDDLE_IDLE')

    def _maybe_alert_pedestrian(self):
        """Speak a warning when a pedestrian is close. Never touches motion."""
        if not self._pedestrian_in_alert_range():
            return
        if self._last_say_stamp is not None and (self.get_clock().now() - self._last_say_stamp) < \
                self._pedestrian_alert_cooldown:
            return
        self._send_say(self._pedestrian_alert_message)

    # ── State transitions ───────────────────────────────────────────────

    def _enter_vehicle_stop(self):
        self._state = State.VEHICLE_STOP
        self.get_logger().info('Vehicle in range — state=VEHICLE_STOP (stop gesture, watching for it to stop)')
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._send_motion(self._motion_init_stop)
        self._send_say(self._vehicle_stop_message)

    def _enter_check_plate(self):
        self._state = State.CHECK_PLATE
        self._check_plate_entered_at = self.get_clock().now()
        self._plate_votes.clear()
        self.get_logger().info('Vehicle stopped — state=CHECK_PLATE (head down, checking plate)')
        self._set_perception_mode(traffic_enabled=False, plate_enabled=True)
        self._send_motion(self._motion_check_plate)

    def _enter_wait_to_leave(self):
        self._state = State.WAIT_TO_LEAVE
        self.get_logger().info('Plate not confirmed in time — state=WAIT_TO_LEAVE (stop gesture, waiting for vehicle to leave)')
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._send_motion(self._motion_init_stop)

    def _enter_cross_vehicle(self):
        self._state = State.CROSS_VEHICLE
        self._pass_gesture_sent = False
        self.get_logger().info('Plate allowed — state=CROSS_VEHICLE (move to pose B, stop-init motion)')
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._send_nav_goal(self._pose_b)
        self._send_motion(self._motion_stop_init)
        self._send_say(self._vehicle_pass_message)

    def _enter_check_vehicle_in_range(self):
        self._state = State.CHECK_VEHICLE_IN_RANGE
        self.get_logger().info('At pose B — state=CHECK_VEHICLE_IN_RANGE (pass gesture, then waving)')

    def _enter_returning(self):
        self._state = State.RETURNING
        self.get_logger().info('Vehicle passed — state=RETURNING (move to pose A + default gesture)')
        self._send_nav_goal(self._pose_a)
        self._send_motion(self._motion_pass_init)

    def _enter_idle(self):
        self._state = State.MIDDLE_IDLE
        self.get_logger().info('Vehicle left range — state=MIDDLE_IDLE (default gesture)')
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._send_motion(self._motion_stop_init)

    def _passing_vehicle(self):
        self.get_logger().info('Plate allowed — state=CHECK_VEHICLE_IN_RANGE (waving)')
        self._send_motion(self._motion_pass_wave)


    # ── Action helpers ───────────────────────────────────────────────────

    def _send_nav_goal(self, pose):
        self._nav_request_id += 1
        request_id = self._nav_request_id

        if not self._nav_done:
            # A previous nav goal is still in flight (executing or being
            # cancelled). Queue this one and cancel the current goal — it
            # gets dispatched once the current goal's *actual result*
            # arrives. The action server rejects a new goal sent right
            # after a cancel is merely acknowledged, before the previous
            # goal has actually finished stopping.
            self._nav_pending = (request_id, pose)
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
            return

        self._nav_pending = None
        self._dispatch_nav_goal(pose, request_id)

    def _dispatch_nav_goal(self, pose, request_id):
        x, y, phi = pose
        goal = GoToXYPhi.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.phi = float(phi)

        self._nav_done = False

        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('go_to_xy_phi action server not available')
            self._nav_done = True
            return

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda fut, rid=request_id: self._nav_goal_response_cb(fut, rid))

    def _maybe_dispatch_pending_nav_goal(self):
        if self._nav_pending is None:
            return
        request_id, pose = self._nav_pending
        self._nav_pending = None
        if request_id == self._nav_request_id:
            self._dispatch_nav_goal(pose, request_id)

    def _send_motion(self, motion_name: str):
        self._motion_request_id += 1
        request_id = self._motion_request_id

        if not self._motion_done:
            # Same reasoning as _send_nav_goal: queue and cancel, then
            # dispatch once the current goal's real result comes back.
            self._motion_pending = (request_id, motion_name)
            if self._motion_goal_handle is not None:
                self._motion_goal_handle.cancel_goal_async()
            return

        self._motion_pending = None
        self._dispatch_motion(motion_name, request_id)

    def _dispatch_motion(self, motion_name, request_id):
        goal = PlayMotion2.Goal()
        goal.motion_name = motion_name
        goal.skip_planning = False

        self._motion_done = False

        if not self._motion_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('play_motion2 action server not available')
            self._motion_done = True
            return

        future = self._motion_client.send_goal_async(goal)
        future.add_done_callback(
            lambda fut, rid=request_id: self._motion_goal_response_cb(fut, rid))

    def _maybe_dispatch_pending_motion(self):
        if self._motion_pending is None:
            return
        request_id, motion_name = self._motion_pending
        self._motion_pending = None
        if request_id == self._motion_request_id:
            self._dispatch_motion(motion_name, request_id)

    def _send_say(self, text: str):
        if self._say_in_flight:
            # Don't overlap TTS goals — vehicle-stop/pass announcements and
            # the pedestrian alert all share this one action client.
            return

        goal = TTS.Goal()
        goal.input = text
        goal.locale = 'en_US'
        goal.voice = ''

        if not self._say_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('TTS action server not available')
            return

        self._say_in_flight = True
        future = self._say_client.send_goal_async(goal)
        future.add_done_callback(self._say_goal_response_cb)

    def _say_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('TTS goal rejected')
            self._say_in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._say_result_cb)

    def _say_result_cb(self, future):
        # Field names on tts_msgs/action/TTS's Result aren't vendored in
        # this workspace to check, so only the generic goal status (always
        # present on any action's WrappedResult) is logged here.
        self.get_logger().info(f'TTS result: status={future.result().status}')
        self._say_in_flight = False
        self._last_say_stamp = self.get_clock().now()

    def _nav_goal_response_cb(self, future, request_id):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('go_to_xy_phi goal rejected')
            self._nav_done = True
            self._maybe_dispatch_pending_nav_goal()
            return
        if request_id != self._nav_request_id:
            # A newer nav goal was requested while this one was in flight —
            # it was accepted too late to be cancelled up front. Cancel it,
            # but still track its result so the pending goal only gets
            # dispatched once this one has actually finished.
            goal_handle.cancel_goal_async()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._nav_result_cb)
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _motion_goal_response_cb(self, future, request_id):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('play_motion2 goal rejected')
            self._motion_done = True
            self._maybe_dispatch_pending_motion()
            return
        if request_id != self._motion_request_id:
            # Same race as above: this motion was superseded before it was
            # even accepted. Cancel it, but still track its result so the
            # pending motion only gets dispatched once it has actually
            # finished — not merely once the cancel is acknowledged.
            goal_handle.cancel_goal_async()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._motion_result_cb)
            return
        self._motion_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._motion_result_cb)

    def _nav_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'go_to_xy_phi result: success={result.success} ({result.message})')
        self._nav_done = True
        self._maybe_dispatch_pending_nav_goal()

    def _motion_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'play_motion2 result: success={result.success} ({result.error})')
        self._motion_done = True
        self._maybe_dispatch_pending_motion()


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
