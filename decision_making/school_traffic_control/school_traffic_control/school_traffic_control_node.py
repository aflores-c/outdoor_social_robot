#!/usr/bin/env python3
"""
School traffic control decision node.

Acts as a crossing-guard state machine: the robot base holds a "middle of
the road" pose while no vehicle needs attention. When a vehicle enters the
close range (range_near_m..range_far_m, default 5-10 m), the robot holds
position and makes a stop gesture while watching for the vehicle to
actually come to a stop (VehicleDetection.stopped, from
traffic_object_detection). Once stopped, the robot looks down at the plate
(head motion) and checks it; if confirmed registered within
plate_confirmation_timeout_s, the robot crosses to pose B, waves the
vehicle through, and returns. If not confirmed in time, the robot goes
back to just holding the stop gesture until the vehicle leaves.

A pedestrian within the alert range is handled separately from the state
machine: the robot only plays an audio clip (robot_audio) and never changes
its base pose or arm gesture because of a pedestrian — traffic control (the
vehicle logic above) remains its only motion-driving task. In MIDDLE_IDLE it
plays pedestrian_introduction_message; in any vehicle-focused state it plays
the plain pedestrian_alert_message instead (see _maybe_alert_pedestrian).

An EMERGENCY state, forced externally via emergency_topic, preempts every
other state: it cancels in-flight nav/motion goals and holds (for teleop)
until the flag clears, then always resumes at MIDDLE_IDLE.

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
  <play_audio_action>     robot_audio_msgs/action/PlayAudio     (voice/audio clips)

State machine (vehicles only):
  MIDDLE_IDLE             -> base at pose A, default arm gesture. Waiting for a
                             vehicle in the close range.
  VEHICLE_STOP            -> base stays at pose A, stop gesture. Traffic detection
                             stays on here (need VehicleDetection.stopped) —
                             waiting for the vehicle to actually come to a stop.
                             -> MIDDLE_IDLE if it leaves the range first.
  CHECK_PLATE             -> head motion looks down at the plate first; only once
                             that motion finishes does plate detection turn on and
                             the plate_confirmation_timeout_s timer start.
                             -> CROSS_VEHICLE if the plate is confirmed in time.
                             -> WAIT_TO_LEAVE if the timer expires first.
  WAIT_TO_LEAVE            -> head motion back up first; once that finishes, the
                             stop gesture is sent and the vehicle-left check begins
                             (traffic detection back on, plate detection off).
                             -> MIDDLE_IDLE once the vehicle leaves the range.
  CROSS_VEHICLE            -> base moves to pose B while the head turns left; once
                             the head motion finishes, the pass gesture plays.
                             -> CHECK_VEHICLE_IN_RANGE once the base has arrived AND
                             the head-left + pass sequence has both finished.
  CHECK_VEHICLE_IN_RANGE   -> waves in a loop while the vehicle is still in range
                             (the pass gesture already played during CROSS_VEHICLE).
                             Tracks the specific vehicle (by id) that was
                             authorized at CROSS_VEHICLE entry, not just "any
                             vehicle in range" — a second, not-yet-authorized
                             car queued behind it doesn't block the return.
                             -> RETURNING once that specific vehicle leaves.
  RETURNING                -> base moves back to pose A, sent together with an
                             arm gesture: default/arms_init if only one car was
                             ever in range, or the stop gesture if a second
                             (queued, unauthorized) car was also seen — so it
                             doesn't think it's been waved through too. Once
                             both complete, state returns to MIDDLE_IDLE.
  EMERGENCY                -> reachable from any state via emergency_topic.
                             Cancels in-flight nav/motion goals and holds for
                             teleop; resumes at MIDDLE_IDLE once cleared.
"""

import json
from collections import Counter, deque
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool, String
from traffic_perception_msgs.msg import VehicleDetectionArray, PedestrianDetectionArray
from drone_traffic_perception.msg import VehicleDetectionCounts

from base_navigation.action import GoToXYPhi
from play_motion2_msgs.action import PlayMotion2
from robot_audio_msgs.action import PlayAudio


class State(Enum):
    MIDDLE_IDLE = auto()
    VEHICLE_STOP = auto()
    CHECK_PLATE = auto()
    WAIT_TO_LEAVE = auto()
    CROSS_VEHICLE = auto()
    CHECK_VEHICLE_IN_RANGE = auto()
    RETURNING = auto()
    EMERGENCY = auto()


class SchoolTrafficControlNode(Node):

    def __init__(self):
        super().__init__('school_traffic_control_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('vehicles_topic', '/perception/vehicles')
        self.declare_parameter('pedestrians_topic', '/perception/pedestrians')
        self.declare_parameter('plate_allowed_topic', '/perception/plate_allowed')
        self.declare_parameter('close_proximity_topic', '/perception/close_proximity')
        self.declare_parameter('force_plate_allowed_topic', '/perception/force_plate_allowed')
        self.declare_parameter('drone_vehicle_detections_topic', 'drone_vehicle_detections')
        self.declare_parameter('emergency_topic', '/school_traffic_control/emergency')
        # Field-trial event log (see benchmark_logging) — published
        # unconditionally; benchmark_logging's data_logger_node is the only
        # place that decides whether a trial is active and gates on that.
        self.declare_parameter('benchmark_events_topic', '/benchmark/events')

        self.declare_parameter('go_to_xy_phi_action', 'go_to_xy_phi')
        self.declare_parameter('play_motion2_action', 'play_motion2')
        self.declare_parameter('play_audio_action', 'play_audio')

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
        self.declare_parameter('range_near_m', 5.0)
        self.declare_parameter('range_far_m', 10.0)

        # Debounce for the vehicle actually leaving range: target_vehicle
        # must read as absent continuously for this long before
        # VEHICLE_STOP/WAIT_TO_LEAVE/CHECK_VEHICLE_IN_RANGE treat it as
        # gone — smooths over a single dropped/late detection frame (e.g.
        # right as a vehicle's stopped flag flips) instead of flickering
        # back to MIDDLE_IDLE/RETURNING and immediately back again.
        self.declare_parameter('vehicle_lost_confirm_s', 0.3)

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

        # Pedestrian alert: within this range, play an audio clip — no base
        # or arm motion is triggered by this. In MIDDLE_IDLE,
        # pedestrian_introduction_message plays instead of the plain alert
        # (see _maybe_alert_pedestrian) — once a vehicle is being handled, we
        # focus on vehicles and fall back to the plain caution clip.
        self.declare_parameter('pedestrian_alert_range_m', 10.0)
        self.declare_parameter('pedestrian_alert_message', 'safe_audio.mp3')
        self.declare_parameter('pedestrian_introduction_message', 'presentation_audio.mp3')
        self.declare_parameter('pedestrian_alert_cooldown_s', 5.0)

        # Audio clips on vehicle state transitions (robot_audio filenames).
        self.declare_parameter('vehicle_stop_message', 'stop_audio.mp3')
        self.declare_parameter('vehicle_pass_message', 'enter_audio.mp3')
        # Played instead of vehicle_stop_message in WAIT_TO_LEAVE when a
        # parking space is free outside the school (see _parking_space_free).
        self.declare_parameter('vehicle_stop_goto_message', 'go_to_sfo_audio.mp3')

        # VehicleDetectionCounts (drone_traffic_perception) mode-smoothing:
        # raw_detections is buffered over the trailing parking_count_window_s
        # and the statistical mode of that window is compared against
        # parking_free_threshold — smooths a single noisy frame instead of
        # trusting the drone node's own already-smoothed average/EMA fields.
        self.declare_parameter('parking_free_threshold', 12)
        self.declare_parameter('parking_count_window_s', 1.0)

        # Drone link staleness: no heartbeat topic exists upstream, so this
        # is inferred purely from gaps between VehicleDetectionCounts
        # arrivals. Deliberately NOT message_timeout_s (1.0s) — the drone's
        # own RTMP reconnect cycle is ~1s, which would flap this constantly.
        self.declare_parameter('drone_link_timeout_s', 4.0)

        # Messages older than this are treated as stale/unknown.
        self.declare_parameter('message_timeout_s', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)

        vehicles_topic = self.get_parameter('vehicles_topic').value
        pedestrians_topic = self.get_parameter('pedestrians_topic').value
        plate_allowed_topic = self.get_parameter('plate_allowed_topic').value
        close_proximity_topic = self.get_parameter('close_proximity_topic').value
        force_plate_allowed_topic = self.get_parameter('force_plate_allowed_topic').value
        drone_vehicle_detections_topic = self.get_parameter('drone_vehicle_detections_topic').value
        emergency_topic = self.get_parameter('emergency_topic').value
        benchmark_events_topic = self.get_parameter('benchmark_events_topic').value
        traffic_detection_enabled_topic = self.get_parameter('traffic_detection_enabled_topic').value
        plate_detection_enabled_topic = self.get_parameter('plate_detection_enabled_topic').value

        go_to_xy_phi_action = self.get_parameter('go_to_xy_phi_action').value
        play_motion2_action = self.get_parameter('play_motion2_action').value
        play_audio_action = self.get_parameter('play_audio_action').value

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
        self._motion_default = "norway_init"
        self._motion_arms_init = "norway_arms_init"
        self._motion_stop = "norway_stop"
        self._motion_right_init = "norway_right_init"
        self._motion_pass = "norway_pass"
        self._motion_pass_wave = "norway_pass_wave"
        self._motion_head_down = "norway_head_down"
        self._motion_head_up = "norway_head_up"
        self._motion_head_left = "norway_head_left"
        self._motion_head_front = "norway_head_front"


        self._range_near = float(self.get_parameter('range_near_m').value)
        self._range_far = float(self.get_parameter('range_far_m').value)
        self._vehicle_lost_confirm = Duration(
            seconds=float(self.get_parameter('vehicle_lost_confirm_s').value))
        self._plate_confirmation_timeout = Duration(
            seconds=float(self.get_parameter('plate_confirmation_timeout_s').value))
        self._plate_vote_min_yes = int(self.get_parameter('plate_vote_min_yes').value)
        plate_vote_window = int(self.get_parameter('plate_vote_window').value)

        self._pedestrian_safety_gate = bool(self.get_parameter('pedestrian_safety_gate').value)
        self._pedestrian_zone_m = float(self.get_parameter('pedestrian_zone_m').value)

        self._pedestrian_alert_range = float(self.get_parameter('pedestrian_alert_range_m').value)
        self._pedestrian_alert_message = self.get_parameter('pedestrian_alert_message').value
        self._pedestrian_introduction_message = self.get_parameter('pedestrian_introduction_message').value
        self._pedestrian_alert_cooldown = Duration(
            seconds=float(self.get_parameter('pedestrian_alert_cooldown_s').value))

        self._vehicle_stop_message = self.get_parameter('vehicle_stop_message').value
        self._vehicle_pass_message = self.get_parameter('vehicle_pass_message').value
        self._vehicle_stop_goto_message = self.get_parameter('vehicle_stop_goto_message').value

        self._parking_free_threshold = int(self.get_parameter('parking_free_threshold').value)
        self._parking_count_window = Duration(
            seconds=float(self.get_parameter('parking_count_window_s').value))

        self._drone_link_timeout = Duration(
            seconds=float(self.get_parameter('drone_link_timeout_s').value))

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

        # Manual override: force the next plate check to pass, for when the
        # plate-OCR pipeline isn't working. One-shot — consumed by _plate_ok
        # the moment it's used, not latched across multiple vehicles.
        self._force_plate_allowed = False

        # Base-scan (SICK front+rear, /scan) close-proximity signal — OR'd
        # with the camera+velodyne pedestrian check, see
        # _pedestrian_in_alert_range. Published by base_scan_proximity.
        self._close_proximity = False
        self._close_proximity_stamp = None

        # (stamp, raw_detections) samples from VehicleDetectionCounts,
        # trimmed to the trailing parking_count_window_s on read — see
        # _count_mode/_parking_space_free.
        self._vehicle_counts_buffer = deque()

        # Drone-link staleness tracking (no heartbeat exists upstream — see
        # _tick's staleness check and drone_link_timeout_s). Starts False
        # (unknown/no data yet) rather than True, so the first
        # VehicleDetectionCounts ever received cleanly fires
        # drone_link_restored ("link established") instead of silently
        # assuming a link that was never confirmed; a drone that's never
        # connected at all (drone_unavailable scenario) then correctly
        # never crosses an edge, i.e. no spurious lost/restored events.
        self._last_vehicle_counts_stamp = None
        self._drone_link_up = False

        # Forces the EMERGENCY state from anywhere; cleared -> MIDDLE_IDLE.
        self._emergency = False

        self.create_subscription(VehicleDetectionArray, vehicles_topic, self._vehicles_cb, 10)
        self.create_subscription(PedestrianDetectionArray, pedestrians_topic, self._pedestrians_cb, 10)
        self.create_subscription(Bool, plate_allowed_topic, self._plate_cb, 10)
        self.create_subscription(Bool, close_proximity_topic, self._close_proximity_cb, 10)
        self.create_subscription(Bool, force_plate_allowed_topic, self._force_plate_cb, 10)
        self.create_subscription(Bool, emergency_topic, self._emergency_cb, 10)
        self.create_subscription(
            VehicleDetectionCounts, drone_vehicle_detections_topic, self._vehicle_counts_cb, 10)

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

        # Field-trial event log (see benchmark_logging/data_logger_node) —
        # published unconditionally; this node doesn't know or care whether
        # a trial is currently active, that's data_logger_node's job.
        self._pub_benchmark_events = self.create_publisher(String, benchmark_events_topic, 10)

        # ── Action clients ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, GoToXYPhi, go_to_xy_phi_action)
        self._motion_client = ActionClient(self, PlayMotion2, play_motion2_action)
        self._audio_client = ActionClient(self, PlayAudio, play_audio_action)

        self._nav_goal_handle = None
        self._motion_goal_handle = None
        self._audio_goal_handle = None

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

        # Used only while in CROSS_VEHICLE, to sequence the pass gesture
        # after the head-left motion finishes instead of sending both at
        # once (a single in-flight motion goal can't be queued twice).
        self._pass_gesture_sent = False

        # Used only while in WAIT_TO_LEAVE, to sequence the stop gesture
        # after the head-up motion finishes, same reasoning as above.
        self._wait_to_leave_stop_sent = False

        # Set once the CHECK_PLATE head-down motion finishes and plate
        # checking actually starts; used both as a "have we started
        # checking yet" gate and for the plate_confirmation_timeout_s
        # fallback to WAIT_TO_LEAVE.
        self._check_plate_entered_at = None

        # Timestamp of when target_vehicle first read as absent, used by
        # _vehicle_confirmed_gone to debounce a single dropped detection
        # frame before actually treating the vehicle as gone. Reset to
        # None whenever target_vehicle is seen present again, and on entry
        # to each state that checks it (_enter_vehicle_stop,
        # _enter_wait_to_leave).
        self._vehicle_lost_since = None

        # CROSS_VEHICLE/CHECK_VEHICLE_IN_RANGE two-car queue tracking. The id
        # of the vehicle authorized/crossing (captured at CROSS_VEHICLE
        # entry; -1 if the detector doesn't support tracking, in which case
        # _crossing_vehicle_confirmed_gone falls back to the plain
        # closest-vehicle-absent check). _had_second_vehicle records whether
        # a second (queued, unauthorized) vehicle was ever seen in range
        # during this crossing, so RETURNING knows which gesture to use.
        # _vehicle_id_lost_since is a debounce timer, separate from
        # _vehicle_lost_since above, for that specific tracked id's absence.
        self._crossing_vehicle_id = -1
        self._had_second_vehicle = False
        self._vehicle_id_lost_since = None

        # Set True on CROSS_VEHICLE entry; while True, _audio_result_cb
        # keeps re-sending vehicle_pass_message until the state changes.
        self._cross_vehicle_looping = False

        # Audio playback bookkeeping (independent of the state machine).
        self._audio_in_flight = False
        self._last_audio_stamp = None

        # ── Field-trial event-log bookkeeping (see _log_event) ──────────────
        # State this _enter_* method transitioned FROM — set at the top of
        # every _enter_* method, before it reassigns self._state, since
        # every _enter_* method does that as its first line.
        self._log_prev_state = None
        # Which path _plate_ok() last returned True via ('force' or 'vote'),
        # read by _enter_cross_vehicle for the plate_confirmed event.
        self._last_plate_ok_via = None
        # Which pose a nav goal was sent to ('pose_a'/'pose_b'), read by
        # _nav_result_cb since it has no other way to know.
        self._nav_target_name = None

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

    def _close_proximity_cb(self, msg: Bool):
        self._close_proximity = msg.data
        self._close_proximity_stamp = self.get_clock().now()

    def _force_plate_cb(self, msg: Bool):
        if msg.data and not self._force_plate_allowed:
            self._log_event('force_plate_allowed_set')
        self._force_plate_allowed = msg.data

    def _emergency_cb(self, msg: Bool):
        self._emergency = msg.data

    def _vehicle_counts_cb(self, msg: VehicleDetectionCounts):
        now = self.get_clock().now()
        self._vehicle_counts_buffer.append((now, int(msg.raw_detections)))
        self._last_vehicle_counts_stamp = now

    # ── Perception load switching ────────────────────────────────────────

    def _set_perception_mode(self, traffic_enabled: bool, plate_enabled: bool):
        """Toggle which heavy perception model is allowed to run — the
        Jetson can't comfortably run traffic_object_detection and
        vehicle_plate_detection at once."""
        self._pub_traffic_enabled.publish(Bool(data=traffic_enabled))
        self._pub_plate_enabled.publish(Bool(data=plate_enabled))

    # ── Field-trial event log ────────────────────────────────────────────

    def _log_event(self, trigger: str, **metadata):
        """Publish a structured, timestamped state-machine event for
        field-trial data collection (see benchmark_logging). Published
        unconditionally — benchmark_logging's data_logger_node is the only
        place that decides whether a trial is currently active."""
        payload = {
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'prev_state': self._log_prev_state.name if self._log_prev_state else None,
            'new_state': self._state.name,
            'trigger': trigger,
            **metadata,
        }
        self._pub_benchmark_events.publish(String(data=json.dumps(payload)))

    # ── Freshness / sensor helpers ──────────────────────────────────────

    def _is_fresh(self, stamp: Time) -> bool:
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) < self._msg_timeout

    def _vehicles_in_range(self):
        if self._vehicles is None or not self._is_fresh(self._vehicles_stamp):
            return []
        return [v for v in self._vehicles if self._range_near <= v.distance <= self._range_far]

    def _closest_vehicle_in_range(self):
        candidates = self._vehicles_in_range()
        if not candidates:
            return None
        return min(candidates, key=lambda v: v.distance)

    def _vehicle_confirmed_gone(self, target_vehicle) -> bool:
        """True once target_vehicle has read as absent continuously for at
        least vehicle_lost_confirm_s — debounces a single dropped/late
        detection frame (e.g. right as a vehicle's stopped flag flips) so
        VEHICLE_STOP/WAIT_TO_LEAVE don't flicker back to MIDDLE_IDLE and
        immediately re-enter."""
        if target_vehicle is not None:
            self._vehicle_lost_since = None
            return False
        if self._vehicle_lost_since is None:
            self._vehicle_lost_since = self.get_clock().now()
            return False
        return (self.get_clock().now() - self._vehicle_lost_since) >= self._vehicle_lost_confirm

    def _crossing_vehicle_confirmed_gone(self) -> bool:
        """CHECK_VEHICLE_IN_RANGE-specific version of _vehicle_confirmed_gone:
        tracks the specific vehicle captured at CROSS_VEHICLE entry
        (self._crossing_vehicle_id), not "current closest in-range vehicle"
        — so a second, not-yet-authorized car queued behind it doesn't make
        the robot think the first (authorized) car is still there. Falls
        back to the plain closest-vehicle-absent check when id tracking
        isn't available (-1)."""
        if self._crossing_vehicle_id == -1:
            return self._vehicle_confirmed_gone(self._closest_vehicle_in_range())

        candidates = self._vehicles_in_range()
        if len(candidates) >= 2 and not self._had_second_vehicle:
            self._had_second_vehicle = True
            self._log_event('second_vehicle_queued')
        present = any(v.id == self._crossing_vehicle_id for v in candidates)

        if present:
            self._vehicle_id_lost_since = None
            return False
        if self._vehicle_id_lost_since is None:
            self._vehicle_id_lost_since = self.get_clock().now()
            return False
        return (self.get_clock().now() - self._vehicle_id_lost_since) >= self._vehicle_lost_confirm

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
        readings) instead of trusting only the single latest message. Also
        true (and self-clearing) if force_plate_allowed_topic forced this
        one vehicle through — for when the OCR pipeline itself is down."""
        if self._force_plate_allowed:
            self._force_plate_allowed = False  # one-shot: consumed by this authorization
            self._last_plate_ok_via = 'force'
            return True
        if not self._is_fresh(self._plate_stamp):
            return False
        if sum(self._plate_votes) >= self._plate_vote_min_yes:
            self._last_plate_ok_via = 'vote'
            return True
        return False

    def _count_mode(self):
        """Statistical mode of raw_detections over the trailing
        parking_count_window_s — smooths a single noisy drone-detector
        frame instead of trusting its own already-smoothed average/EMA
        fields. None if no sample has arrived within the window."""
        cutoff = self.get_clock().now() - self._parking_count_window
        while self._vehicle_counts_buffer and self._vehicle_counts_buffer[0][0] < cutoff:
            self._vehicle_counts_buffer.popleft()
        if not self._vehicle_counts_buffer:
            return None
        counts = [c for _, c in self._vehicle_counts_buffer]
        return Counter(counts).most_common(1)[0][0]

    def _parking_space_free(self) -> bool:
        mode = self._count_mode()
        if mode is None:
            return False
        return mode < self._parking_free_threshold

    def _pedestrian_in_alert_range(self) -> bool:
        camera_lidar = (
            self._pedestrians is not None and self._is_fresh(self._pedestrians_stamp)
            and any(p.distance <= self._pedestrian_alert_range for p in self._pedestrians))
        scan = self._close_proximity and self._is_fresh(self._close_proximity_stamp)
        return camera_lidar or scan

    # ── Control loop / state machine ────────────────────────────────────

    def _tick(self):
        # Checked before anything else, every tick, from any state — a
        # forced emergency preempts autonomous behavior entirely so the
        # robot can be teleoperated. Placed ahead of
        # _maybe_retry_pending_goals so a goal queued right before the
        # emergency can't sneak out on the same tick its cancel is issued.
        if self._emergency:
            if self._state != State.EMERGENCY:
                self._enter_emergency()
            return
        if self._state == State.EMERGENCY:
            self._enter_idle(reason='Emergency cleared')
            return

        self._maybe_retry_pending_goals()
        self._maybe_alert_pedestrian()
        self._maybe_check_drone_link()

        target_vehicle = self._closest_vehicle_in_range()

        if self._state == State.MIDDLE_IDLE:
            if target_vehicle is not None:
                self._enter_vehicle_stop()

        elif self._state == State.VEHICLE_STOP:
            # Traffic detection stays on for this whole state (unlike
            # CHECK_PLATE), so target_vehicle is always live here — no
            # staleness caveat needed. _vehicle_confirmed_gone debounces a
            # single dropped/late detection frame (common right as a
            # vehicle's stopped flag flips) so it doesn't bounce back to
            # MIDDLE_IDLE mid-interaction.
            if self._vehicle_confirmed_gone(target_vehicle):
                self._enter_idle()
            elif target_vehicle is not None and target_vehicle.stopped:
                self._enter_check_plate()

        elif self._state == State.CHECK_PLATE:
            if self._check_plate_entered_at is None:
                # Still turning the head down — don't start trusting plate
                # readings (or the confirmation timeout) until it's done,
                # since the camera isn't pointed at the plate yet.
                if self._motion_done:
                    self._check_plate_entered_at = self.get_clock().now()
                    self._plate_votes.clear()
                    self._set_perception_mode(traffic_enabled=False, plate_enabled=True)
            elif self._plate_ok():
                self._enter_cross_vehicle(target_vehicle)
            elif (self.get_clock().now() - self._check_plate_entered_at) > self._plate_confirmation_timeout:
                self._enter_wait_to_leave()

        elif self._state == State.WAIT_TO_LEAVE:
            if not self._motion_done and not self._wait_to_leave_stop_sent:
                # Still raising the head back up.
                pass
            else:
                if not self._wait_to_leave_stop_sent:
                    self._wait_to_leave_stop_sent = True
                    self._send_motion(self._motion_stop)
                    self._send_audio(self._vehicle_stop_goto_message if self._parking_space_free()
                                      else self._vehicle_stop_message)
                if self._vehicle_confirmed_gone(target_vehicle):
                    self._enter_idle()

        elif self._state == State.CROSS_VEHICLE:
            if not self._motion_done:
                # Still turning the head left or playing the pass gesture —
                # wait for each to finish before moving to the next step.
                pass
            elif not self._pass_gesture_sent:
                self._pass_gesture_sent = True
                self._send_motion(self._motion_pass)
            elif self._nav_done:
                self._enter_check_vehicle_in_range()

        elif self._state == State.CHECK_VEHICLE_IN_RANGE:
            if self._crossing_vehicle_confirmed_gone():
                # The specific vehicle that was authorized/crossing has
                # passed through — head back immediately, even if a second,
                # not-yet-authorized vehicle is still queued in range.
                self._enter_returning()
            elif not self._motion_done:
                # Still finishing the previous wave cycle — wait for it
                # before sending the next one instead of piling goals on
                # top of it.
                pass
            elif target_vehicle is not None:
                # Keep waving while the vehicle is in range: only send a
                # new wave once the previous one has finished, not on
                # every 10 Hz tick.
                self._passing_vehicle()

        elif self._state == State.RETURNING:
            if self._nav_done and self._motion_done:
                self._log_prev_state = self._state
                self._state = State.MIDDLE_IDLE
                self._log_event('returned_to_pose_a')
                self.get_logger().info('Back at middle pose — state=MIDDLE_IDLE')

    def _maybe_retry_pending_goals(self):
        # Only handles the "action server wasn't discovered yet" case: here
        # _*_done is still True since no goal ever actually went in flight.
        # The busy-goal case (_*_done False) already retries on its own via
        # _nav_result_cb/_motion_result_cb once the current goal finishes.
        if self._nav_pending is not None and self._nav_done:
            self._maybe_dispatch_pending_nav_goal()
        if self._motion_pending is not None and self._motion_done:
            self._maybe_dispatch_pending_motion()

    def _maybe_alert_pedestrian(self):
        """Play a warning/introduction clip when a pedestrian is close.
        Never touches motion. In MIDDLE_IDLE plays the friendlier
        introduction clip; once a vehicle is being handled (any other
        state), falls back to the plain caution clip — "when a vehicle
        approaches, we focus on vehicles"."""
        # Inlined from _pedestrian_in_alert_range's two sub-checks, purely
        # so the triggering source can be logged — doesn't change that
        # method's own return contract.
        camera_lidar = (
            self._pedestrians is not None and self._is_fresh(self._pedestrians_stamp)
            and any(p.distance <= self._pedestrian_alert_range for p in self._pedestrians))
        scan = self._close_proximity and self._is_fresh(self._close_proximity_stamp)
        if not (camera_lidar or scan):
            return
        if self._last_audio_stamp is not None and (self.get_clock().now() - self._last_audio_stamp) < \
                self._pedestrian_alert_cooldown:
            return
        if self._state == State.MIDDLE_IDLE:
            message = self._pedestrian_introduction_message
            trigger = 'pedestrian_introduction'
        else:
            message = self._pedestrian_alert_message
            trigger = 'pedestrian_alert'
        self._log_event(trigger, camera_lidar=camera_lidar, scan=scan)
        self._send_audio(message)

    def _maybe_check_drone_link(self):
        """Edge-detected drone-link staleness — no heartbeat topic exists
        upstream, so "link down" is inferred purely from a gap between
        VehicleDetectionCounts arrivals exceeding drone_link_timeout_s.
        This is new instrumentation logic, not a pre-existing signal."""
        if self._last_vehicle_counts_stamp is None:
            return  # no data ever received yet — see _drone_link_up's init comment
        stale = (self.get_clock().now() - self._last_vehicle_counts_stamp) > self._drone_link_timeout
        if stale and self._drone_link_up:
            self._drone_link_up = False
            self._log_event('drone_link_lost')
        elif not stale and not self._drone_link_up:
            self._drone_link_up = True
            self._log_event('drone_link_restored')

    # ── State transitions ───────────────────────────────────────────────

    def _enter_vehicle_stop(self):
        self._log_prev_state = self._state
        self._state = State.VEHICLE_STOP
        self._vehicle_lost_since = None
        self.get_logger().info('Vehicle in range — state=VEHICLE_STOP (stop gesture, watching for it to stop)')
        self._log_event('vehicle_detected')
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._send_motion(self._motion_stop)
        self._send_audio(self._vehicle_stop_message)

    def _enter_check_plate(self):
        self._log_prev_state = self._state
        self._state = State.CHECK_PLATE
        # Left None until the head-down motion finishes — see _tick, which
        # starts the actual plate checking (perception mode + votes +
        # timeout timer) only once that happens.
        self._check_plate_entered_at = None
        self.get_logger().info('Vehicle stopped — state=CHECK_PLATE (head down, checking plate)')
        self._log_event('vehicle_stopped_confirmed')
        self._send_motion(self._motion_head_down)

    def _enter_wait_to_leave(self):
        self._log_prev_state = self._state
        self._state = State.WAIT_TO_LEAVE
        self._wait_to_leave_stop_sent = False
        self._vehicle_lost_since = None
        self.get_logger().info('Plate not confirmed in time — state=WAIT_TO_LEAVE (head up, then stop gesture, waiting for vehicle to leave)')
        self._log_event('plate_confirmation_timeout')
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._send_motion(self._motion_head_up)

    def _enter_cross_vehicle(self, target_vehicle):
        self._log_prev_state = self._state
        self._state = State.CROSS_VEHICLE
        self._pass_gesture_sent = False
        # Two-car queue tracking (see _crossing_vehicle_confirmed_gone):
        # remember which vehicle this crossing authorizes, and reset the
        # per-crossing queue bookkeeping fresh for this car.
        self._crossing_vehicle_id = target_vehicle.id if target_vehicle is not None else -1
        self._had_second_vehicle = False
        self._vehicle_id_lost_since = None
        self.get_logger().info('Plate allowed — state=CROSS_VEHICLE (move to pose B, head left)')
        self._log_event('plate_confirmed', via=self._last_plate_ok_via)
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._nav_target_name = 'pose_b'
        self._log_event('nav_goal_sent', target='pose_b')
        self._send_nav_goal(self._pose_b)
        self._send_motion(self._motion_head_left)
        self._send_motion(self._motion_right_init)
        # Looped for the whole state — see _audio_result_cb.
        self._cross_vehicle_looping = True
        self._send_audio(self._vehicle_pass_message)

    def _enter_check_vehicle_in_range(self):
        self._log_prev_state = self._state
        self._state = State.CHECK_VEHICLE_IN_RANGE
        self._vehicle_lost_since = None
        self._stop_cross_vehicle_audio_loop()
        self.get_logger().info('At pose B — state=CHECK_VEHICLE_IN_RANGE (pass gesture, then waving)')
        self._log_event('arrived_pose_b')

    def _enter_returning(self):
        self._log_prev_state = self._state
        self._state = State.RETURNING
        self.get_logger().info('Vehicle passed — state=RETURNING (move to pose A + gesture)')
        self._log_event('crossing_vehicle_gone')
        self._nav_target_name = 'pose_a'
        self._log_event('nav_goal_sent', target='pose_a')
        self._send_nav_goal(self._pose_a)
        self._send_motion(self._motion_head_front)
        # A second (queued, unauthorized) car was in range at some point
        # during the crossing -> keep the stop gesture so it doesn't think
        # it's been waved through too. Otherwise, default/arms_init.
        self._send_motion(self._motion_stop if self._had_second_vehicle else self._motion_arms_init)

    def _enter_idle(self, reason: str = 'Vehicle left range'):
        self._log_prev_state = self._state
        self._state = State.MIDDLE_IDLE
        self.get_logger().info(f'{reason} — state=MIDDLE_IDLE (default gesture)')
        self._log_event('vehicle_left_range' if reason == 'Vehicle left range' else 'emergency_cleared')
        self._set_perception_mode(traffic_enabled=True, plate_enabled=False)
        self._send_motion(self._motion_arms_init)

    def _enter_emergency(self):
        self._log_prev_state = self._state
        self._state = State.EMERGENCY
        self.get_logger().warn('Emergency flag set — state=EMERGENCY (canceling autonomous goals, holding for teleop)')
        self._log_event('emergency_triggered')
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
        if self._motion_goal_handle is not None:
            self._motion_goal_handle.cancel_goal_async()
        self._nav_pending = None
        self._motion_pending = None
        self._stop_cross_vehicle_audio_loop()

    def _passing_vehicle(self):
        self.get_logger().info('Plate allowed — state=CHECK_VEHICLE_IN_RANGE (waving)')
        self._send_motion(self._motion_pass_wave)

    def _stop_cross_vehicle_audio_loop(self):
        self._cross_vehicle_looping = False
        if self._audio_goal_handle is not None:
            self._audio_goal_handle.cancel_goal_async()


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
        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            # Not discovered yet (e.g. right at startup, before cross-machine
            # DDS discovery has caught up) — keep it queued and let _tick's
            # _maybe_retry_pending_goals try again on the next cycle, rather
            # than silently dropping this goal forever.
            self.get_logger().warn('go_to_xy_phi action server not available, will retry',
                                    throttle_duration_sec=2.0)
            self._nav_pending = (request_id, pose)
            return

        x, y, phi = pose
        goal = GoToXYPhi.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.phi = float(phi)

        self._nav_done = False

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
        if not self._motion_client.wait_for_server(timeout_sec=0.0):
            # Same reasoning as _dispatch_nav_goal: keep it queued and retry
            # from _tick instead of dropping it.
            self.get_logger().warn('play_motion2 action server not available, will retry',
                                    throttle_duration_sec=2.0)
            self._motion_pending = (request_id, motion_name)
            return

        goal = PlayMotion2.Goal()
        goal.motion_name = motion_name
        goal.skip_planning = False

        self._motion_done = False

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

    def _send_audio(self, file_name: str):
        if self._audio_in_flight:
            # Don't overlap audio goals — vehicle-stop/pass announcements
            # and the pedestrian alert all share this one action client.
            return

        goal = PlayAudio.Goal()
        goal.file_name = file_name

        if not self._audio_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('play_audio action server not available')
            return

        self._audio_in_flight = True
        future = self._audio_client.send_goal_async(goal)
        future.add_done_callback(self._audio_goal_response_cb)

    def _audio_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('play_audio goal rejected')
            self._audio_in_flight = False
            return
        self._audio_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._audio_result_cb)

    def _audio_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'play_audio result: success={result.success} ({result.message})')
        self._audio_in_flight = False
        self._audio_goal_handle = None
        self._last_audio_stamp = self.get_clock().now()
        # CROSS_VEHICLE loops vehicle_pass_message for the whole state —
        # re-trigger here as long as we're still in it (see
        # _enter_cross_vehicle / _stop_cross_vehicle_audio_loop).
        if self._cross_vehicle_looping and self._state == State.CROSS_VEHICLE:
            self._send_audio(self._vehicle_pass_message)

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
        self._log_event('nav_goal_result', target=self._nav_target_name,
                         success=result.success, message=result.message)
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
