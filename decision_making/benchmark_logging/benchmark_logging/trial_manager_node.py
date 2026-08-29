#!/usr/bin/env python3
"""
Field-trial start/stop manager.

Owns the trial control/status bus and the per-trial raw-sensor recording
(a throttled-camera + optional-lidar rosbag2 capture). Does not do any
JSONL logging itself — that's data_logger_node's job; this node only
decides when a trial is active and what gets bag-recorded.

Inputs:
  <trial_control_topic>  std_msgs/String  (default /benchmark/trial_control)
                          JSON: {"cmd":"start","scenario_type":"...",
                                 "trial_id":"optional","record_lidar":false}
                          or    {"cmd":"stop"}
  <camera_compressed_topic>  sensor_msgs/CompressedImage  (default
                          /camera/realsense2_camera/color/image_raw/compressed)
                          — republished throttled while a trial is active.

Outputs:
  <current_trial_topic>  std_msgs/String  (default /benchmark/current_trial,
                          TRANSIENT_LOCAL so late subscribers get current
                          state immediately)
                          JSON: {"trial_id":..., "scenario_type":..., "active": bool}
  /benchmark/camera_throttled/compressed  sensor_msgs/CompressedImage
                          — rate-gated republish of the real camera topic,
                          only while a trial is active. Recorded into the
                          trial's rosbag2 bag (not topic_tools — not
                          installed in this environment, a plain
                          monotonic-time gate does the same job in-process).

Each trial's raw recording lives at <bag_output_dir>/<trial_id>/bag
(rosbag2), containing the throttled camera topic and, if
record_lidar was requested, /velodyne_points.
"""

import json
import os
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# rosbag2 needs a clean shutdown (SIGINT) to flush its sqlite3/mcap index —
# a bare SIGKILL risks a truncated/unreadable bag. This is how long to wait
# for that clean shutdown before falling back to SIGKILL.
_BAG_STOP_GRACE_S = 5.0


class TrialManagerNode(Node):

    def __init__(self):
        super().__init__('trial_manager_node')

        self.declare_parameter('trial_control_topic', '/benchmark/trial_control')
        self.declare_parameter('current_trial_topic', '/benchmark/current_trial')
        self.declare_parameter('camera_compressed_topic',
                                '/camera/realsense2_camera/color/image_raw/compressed')
        self.declare_parameter('camera_throttled_topic', '/benchmark/camera_throttled/compressed')
        self.declare_parameter('camera_throttle_hz', 5.0)
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('bag_output_dir', os.path.expanduser('~/benchmark_data'))

        trial_control_topic = self.get_parameter('trial_control_topic').value
        current_trial_topic = self.get_parameter('current_trial_topic').value
        camera_compressed_topic = self.get_parameter('camera_compressed_topic').value
        self._camera_throttled_topic = self.get_parameter('camera_throttled_topic').value
        throttle_hz = float(self.get_parameter('camera_throttle_hz').value)
        self._throttle_period = 1.0 / throttle_hz if throttle_hz > 0 else 0.0
        self._lidar_topic = self.get_parameter('lidar_topic').value
        self._bag_output_dir = os.path.expanduser(self.get_parameter('bag_output_dir').value)

        self._active = False
        self._trial_id = None
        self._scenario_type = None
        self._record_lidar = False
        self._bag_proc = None
        self._last_throttle_t = 0.0

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_current_trial = self.create_publisher(String, current_trial_topic, latched_qos)
        self._pub_camera_throttled = self.create_publisher(CompressedImage, self._camera_throttled_topic, 10)

        self.create_subscription(String, trial_control_topic, self._on_trial_control, 10)
        self.create_subscription(CompressedImage, camera_compressed_topic, self._on_camera_frame, 10)

        self._publish_current_trial()
        self.get_logger().info(
            f"trial_manager_node ready — control='{trial_control_topic}', "
            f"status='{current_trial_topic}', bag_output_dir='{self._bag_output_dir}'")

    # ── Trial control ────────────────────────────────────────────────────

    def _on_trial_control(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f"trial_control: invalid JSON ({e}): {msg.data!r}")
            return

        action = cmd.get('cmd')
        if action == 'start':
            self._start_trial(cmd)
        elif action == 'stop':
            self._stop_trial()
        else:
            self.get_logger().warn(f"trial_control: unknown cmd {action!r}")

    def _start_trial(self, cmd: dict):
        if self._active:
            self.get_logger().warn(
                f"start requested while trial '{self._trial_id}' is still active — "
                f"stopping it first")
            self._stop_trial()

        scenario_type = cmd.get('scenario_type', 'unspecified')
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        trial_id = cmd.get('trial_id') or f'{timestamp}_{scenario_type}_{uuid.uuid4().hex[:6]}'
        record_lidar = bool(cmd.get('record_lidar', False))

        self._trial_id = trial_id
        self._scenario_type = scenario_type
        self._record_lidar = record_lidar
        self._active = True
        self._last_throttle_t = 0.0

        bag_path = os.path.join(self._bag_output_dir, trial_id, 'bag')
        os.makedirs(os.path.dirname(bag_path), exist_ok=True)
        topics = [self._camera_throttled_topic]
        if record_lidar:
            topics.append(self._lidar_topic)

        try:
            self._bag_proc = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_path] + topics,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().error(f"failed to start ros2 bag record: {e}")
            self._bag_proc = None

        self._publish_current_trial()
        self.get_logger().info(
            f"trial started: id='{trial_id}' scenario='{scenario_type}' "
            f"record_lidar={record_lidar} bag='{bag_path}'")

    def _stop_trial(self):
        if not self._active:
            self.get_logger().warn("stop requested but no trial is active")
            return

        if self._bag_proc is not None:
            self._bag_proc.send_signal(signal.SIGINT)
            try:
                self._bag_proc.wait(timeout=_BAG_STOP_GRACE_S)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    f"ros2 bag record didn't exit within {_BAG_STOP_GRACE_S}s — "
                    f"SIGKILL (bag may be truncated)")
                self._bag_proc.kill()
                self._bag_proc.wait()
            self._bag_proc = None

        self.get_logger().info(f"trial stopped: id='{self._trial_id}'")
        self._active = False
        self._publish_current_trial()

    def _publish_current_trial(self):
        payload = {
            'trial_id': self._trial_id,
            'scenario_type': self._scenario_type,
            'active': self._active,
        }
        self._pub_current_trial.publish(String(data=json.dumps(payload)))

    # ── Throttled camera republish (no topic_tools dependency) ─────────────

    def _on_camera_frame(self, msg: CompressedImage):
        if not self._active:
            return
        now = time.monotonic()
        if self._throttle_period > 0.0 and (now - self._last_throttle_t) < self._throttle_period:
            return
        self._last_throttle_t = now
        self._pub_camera_throttled.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrialManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
