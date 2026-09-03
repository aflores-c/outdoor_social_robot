#!/usr/bin/env python3
"""
Field-trial JSONL logger.

Subscribes to everything relevant for the paper's evaluation and writes
one JSON-lines file per stream, per trial, under
<log_dir>/<trial_id>/{events,detections,plate_reads,drone,poses}.jsonl.
Nothing is written while no trial is active (gated by trial_manager_node's
current_trial status) — this is deliberately not one continuous
undifferentiated log.

Inputs:
  <current_trial_topic>   std_msgs/String  (default /benchmark/current_trial,
                           TRANSIENT_LOCAL — see trial_manager_node)
  <events_topic>           std_msgs/String  (default /benchmark/events,
                           JSON — published by school_traffic_control_node)
  <vehicles_topic>         traffic_perception_msgs/VehicleDetectionArray
  <pedestrians_topic>      traffic_perception_msgs/PedestrianDetectionArray
  <plate_result_topic>     traffic_perception_msgs/PlateResult
  <close_proximity_topic>  std_msgs/Bool
  <drone_counts_topic>     drone_traffic_perception/VehicleDetectionCounts
  <drone_link_status_topic>  drone_traffic_perception/DroneLinkStatus
  <pose_topic>              geometry_msgs/PoseWithCovarianceStamped (default /amcl_pose)
  <gps_topic>               sensor_msgs/NavSatFix (default /fix, sparkfun_rtk_gps_bringup)
  <imu_topic>               sensor_msgs/Imu (default /imu/data, xsens_mti_imu_bringup) —
                           throttled by imu_log_hz, same reasoning as pose_log_hz:
                           this publishes far faster than is useful to log every frame of.

Log schemas (one JSON object per line):
  events.jsonl:      {stamp, trial_id, prev_state, new_state, trigger, ...metadata}
  detections.jsonl:  {stamp, kind: "vehicle"|"pedestrian"|"close_proximity", ...}
  plate_reads.jsonl: {stamp, plate_text, det_confidence, ocr_confidence, authorized}
  drone.jsonl:       {stamp, kind: "counts"|"link_status", ...}
  poses.jsonl:       {stamp, x, y, yaw_deg, cov_xx, cov_yy}
  gps.jsonl:         {stamp, latitude, longitude, altitude, status, cov_xx, cov_yy, cov_zz}
  imu.jsonl:         {stamp, yaw_deg, angular_velocity: {x,y,z}, linear_acceleration: {x,y,z}}
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from traffic_perception_msgs.msg import (
    VehicleDetectionArray, PedestrianDetectionArray, PlateResult,
)
from drone_traffic_perception.msg import VehicleDetectionCounts, DroneLinkStatus

_LOG_NAMES = ('events', 'detections', 'plate_reads', 'drone', 'poses', 'gps', 'imu')


def _yaw_deg_from_quaternion(q) -> float:
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.degrees(yaw)


class DataLoggerNode(Node):

    def __init__(self):
        super().__init__('data_logger_node')

        self.declare_parameter('current_trial_topic', '/benchmark/current_trial')
        self.declare_parameter('events_topic', '/benchmark/events')
        self.declare_parameter('vehicles_topic', '/perception/vehicles')
        self.declare_parameter('pedestrians_topic', '/perception/pedestrians')
        self.declare_parameter('plate_result_topic', '/perception/plate_result')
        self.declare_parameter('close_proximity_topic', '/perception/close_proximity')
        self.declare_parameter('drone_counts_topic', 'drone_vehicle_detections')
        self.declare_parameter('drone_link_status_topic', 'drone_vehicle_detections_link_status')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('pose_log_hz', 2.0)
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('gps_log_hz', 0.0)  # 0 = log every message, GPS is already low-rate
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('imu_log_hz', 10.0)  # IMU publishes much faster than useful to log raw
        self.declare_parameter('log_dir', os.path.expanduser('~/benchmark_data'))

        current_trial_topic = self.get_parameter('current_trial_topic').value
        events_topic = self.get_parameter('events_topic').value
        vehicles_topic = self.get_parameter('vehicles_topic').value
        pedestrians_topic = self.get_parameter('pedestrians_topic').value
        plate_result_topic = self.get_parameter('plate_result_topic').value
        close_proximity_topic = self.get_parameter('close_proximity_topic').value
        drone_counts_topic = self.get_parameter('drone_counts_topic').value
        drone_link_status_topic = self.get_parameter('drone_link_status_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        pose_log_hz = float(self.get_parameter('pose_log_hz').value)
        self._pose_log_period = 1.0 / pose_log_hz if pose_log_hz > 0 else 0.0
        self._last_pose_log_t = 0.0
        gps_topic = self.get_parameter('gps_topic').value
        gps_log_hz = float(self.get_parameter('gps_log_hz').value)
        self._gps_log_period = 1.0 / gps_log_hz if gps_log_hz > 0 else 0.0
        self._last_gps_log_t = 0.0
        imu_topic = self.get_parameter('imu_topic').value
        imu_log_hz = float(self.get_parameter('imu_log_hz').value)
        self._imu_log_period = 1.0 / imu_log_hz if imu_log_hz > 0 else 0.0
        self._last_imu_log_t = 0.0
        self._log_dir = os.path.expanduser(self.get_parameter('log_dir').value)

        self._active = False
        self._trial_id = None
        self._files = {}  # name -> open file handle, only while a trial is active

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, current_trial_topic, self._on_current_trial, latched_qos)

        self.create_subscription(String, events_topic, self._on_event, 10)
        self.create_subscription(VehicleDetectionArray, vehicles_topic, self._on_vehicles, 10)
        self.create_subscription(PedestrianDetectionArray, pedestrians_topic, self._on_pedestrians, 10)
        self.create_subscription(PlateResult, plate_result_topic, self._on_plate_result, 10)
        self.create_subscription(Bool, close_proximity_topic, self._on_close_proximity, 10)
        self.create_subscription(VehicleDetectionCounts, drone_counts_topic, self._on_drone_counts, 10)
        self.create_subscription(DroneLinkStatus, drone_link_status_topic, self._on_drone_link_status, 10)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self._on_pose, 10)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)

        self.get_logger().info(f"data_logger_node ready — log_dir='{self._log_dir}'")

    # ── Trial lifecycle ──────────────────────────────────────────────────

    def _on_current_trial(self, msg: String):
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f"current_trial: invalid JSON ({e}): {msg.data!r}")
            return

        was_active = self._active
        self._active = bool(status.get('active', False))
        self._trial_id = status.get('trial_id')

        if self._active and not was_active:
            self._open_files()
        elif was_active and not self._active:
            self._close_files()

    def _open_files(self):
        trial_dir = os.path.join(self._log_dir, self._trial_id)
        os.makedirs(trial_dir, exist_ok=True)
        self._files = {
            name: open(os.path.join(trial_dir, f'{name}.jsonl'), 'a', buffering=1)
            for name in _LOG_NAMES
        }
        self.get_logger().info(f"logging trial '{self._trial_id}' to '{trial_dir}'")

    def _close_files(self):
        for f in self._files.values():
            f.flush()
            f.close()
        self._files = {}
        self.get_logger().info(f"stopped logging trial '{self._trial_id}'")

    def _write(self, name: str, record: dict):
        if not self._active:
            return
        record.setdefault('stamp', self.get_clock().now().nanoseconds * 1e-9)
        record.setdefault('trial_id', self._trial_id)
        self._files[name].write(json.dumps(record) + '\n')

    # ── Subscription callbacks ──────────────────────────────────────────

    def _on_event(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f"events: invalid JSON ({e}): {msg.data!r}")
            return
        self._write('events', payload)

    def _on_vehicles(self, msg: VehicleDetectionArray):
        self._write('detections', {
            'kind': 'vehicle',
            'vehicles': [
                {'id': v.id, 'distance': v.distance,
                 'x': v.position.x, 'y': v.position.y, 'z': v.position.z,
                 'stopped': v.stopped, 'confidence': v.confidence}
                for v in msg.vehicles
            ],
        })

    def _on_pedestrians(self, msg: PedestrianDetectionArray):
        self._write('detections', {
            'kind': 'pedestrian',
            'pedestrians': [
                {'id': p.id, 'distance': p.distance,
                 'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                 'confidence': p.confidence}
                for p in msg.pedestrians
            ],
        })

    def _on_close_proximity(self, msg: Bool):
        self._write('detections', {'kind': 'close_proximity', 'close': bool(msg.data)})

    def _on_plate_result(self, msg: PlateResult):
        self._write('plate_reads', {
            'plate_text': msg.plate_text,
            'det_confidence': msg.det_confidence,
            'ocr_confidence': msg.ocr_confidence,
            'authorized': msg.authorized,
        })

    def _on_drone_counts(self, msg: VehicleDetectionCounts):
        self._write('drone', {
            'kind': 'counts',
            'raw_detections': msg.raw_detections,
            'ema_detections': msg.ema_detections,
            'average_detections': msg.average_detections,
        })

    def _on_drone_link_status(self, msg: DroneLinkStatus):
        self._write('drone', {
            'kind': 'link_status',
            'connected': msg.connected,
            'fps': msg.fps,
            'dropped_frames_total': msg.dropped_frames_total,
            'decode_latency_ms': msg.decode_latency_ms,
        })

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        if not self._active:
            return
        now = time.monotonic()
        if self._pose_log_period > 0.0 and (now - self._last_pose_log_t) < self._pose_log_period:
            return
        self._last_pose_log_t = now
        p = msg.pose.pose.position
        cov = msg.pose.covariance  # row-major 6x6
        self._write('poses', {
            'x': p.x, 'y': p.y,
            'yaw_deg': _yaw_deg_from_quaternion(msg.pose.pose.orientation),
            'cov_xx': cov[0], 'cov_yy': cov[7],
        })

    def _on_gps(self, msg: NavSatFix):
        if not self._active:
            return
        now = time.monotonic()
        if self._gps_log_period > 0.0 and (now - self._last_gps_log_t) < self._gps_log_period:
            return
        self._last_gps_log_t = now
        cov = msg.position_covariance  # row-major 3x3
        self._write('gps', {
            'latitude': msg.latitude, 'longitude': msg.longitude, 'altitude': msg.altitude,
            'status': msg.status.status,
            'cov_xx': cov[0], 'cov_yy': cov[4], 'cov_zz': cov[8],
        })

    def _on_imu(self, msg: Imu):
        if not self._active:
            return
        now = time.monotonic()
        if self._imu_log_period > 0.0 and (now - self._last_imu_log_t) < self._imu_log_period:
            return
        self._last_imu_log_t = now
        av = msg.angular_velocity
        la = msg.linear_acceleration
        self._write('imu', {
            'yaw_deg': _yaw_deg_from_quaternion(msg.orientation),
            'angular_velocity': {'x': av.x, 'y': av.y, 'z': av.z},
            'linear_acceleration': {'x': la.x, 'y': la.y, 'z': la.z},
        })


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
