#!/usr/bin/env python3
"""
Operator CLI for starting/stopping a field trial without hand-typing JSON.

  ros2 run benchmark_logging start_trial --scenario-type authorized_car
  ros2 run benchmark_logging start_trial --scenario-type queued_cars --record-lidar
  ros2 run benchmark_logging stop_trial

Both are one-shot: publish once to /benchmark/trial_control, wait briefly
for the publish to actually go out, then exit.
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_SCENARIO_TYPES = (
    'authorized_car', 'unauthorized_car', 'pedestrian_only',
    'car_and_pedestrian', 'queued_cars', 'drone_unavailable',
)


class _OneShotPublisher(Node):

    def __init__(self):
        super().__init__('benchmark_cli')
        self.declare_parameter('trial_control_topic', '/benchmark/trial_control')
        topic = self.get_parameter('trial_control_topic').value
        self._pub = self.create_publisher(String, topic, 10)

    def send(self, payload: dict):
        self._pub.publish(String(data=json.dumps(payload)))
        # Give discovery/publish a moment to actually reach the subscriber
        # before the process exits.
        rclpy.spin_once(self, timeout_sec=1.0)
        time.sleep(0.5)


def start_trial():
    parser = argparse.ArgumentParser(description='Start a field trial.')
    parser.add_argument('--scenario-type', required=True,
                         help=f'One of: {", ".join(_SCENARIO_TYPES)} (or any custom label)')
    parser.add_argument('--trial-id', default=None,
                         help='Optional explicit trial id; auto-generated if omitted')
    parser.add_argument('--record-lidar', action='store_true',
                         help='Also bag-record raw /velodyne_points (large — see storage notes)')
    args = parser.parse_args()

    rclpy.init()
    node = _OneShotPublisher()
    payload = {
        'cmd': 'start',
        'scenario_type': args.scenario_type,
        'record_lidar': args.record_lidar,
    }
    if args.trial_id:
        payload['trial_id'] = args.trial_id
    node.send(payload)
    node.get_logger().info(f"Sent start-trial: {payload}")
    node.destroy_node()
    rclpy.try_shutdown()


def stop_trial():
    parser = argparse.ArgumentParser(description='Stop the currently active field trial.')
    parser.parse_args()

    rclpy.init()
    node = _OneShotPublisher()
    node.send({'cmd': 'stop'})
    node.get_logger().info('Sent stop-trial')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    start_trial()
