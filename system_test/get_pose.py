#!/usr/bin/env python3
"""Print the robot's current x, y, phi_deg (map -> base_link) for filling in
school_traffic_control's pose_a_x/y/phi_deg / pose_b_x/y/phi_deg config.

Usage (on the robot, with the workspace + PAL setup already sourced):
    python3 get_pose.py

Waits up to 30s for the transform to become available before giving up --
a fresh listener needs a moment to discover the TF publishers over DDS, so
a "no transform yet" on the first try doesn't necessarily mean AMCL isn't
converged. Prints a waiting message every 5s so a real hang is obvious.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

TIMEOUT_SEC = 30.0
POLL_SEC = 0.2


def main():
    rclpy.init()
    node = Node('pose_reader')
    buf = Buffer()
    TransformListener(buf, node)

    attempts = int(TIMEOUT_SEC / POLL_SEC)
    tf = None
    for i in range(attempts):
        rclpy.spin_once(node, timeout_sec=POLL_SEC)
        if i > 0 and i % int(5.0 / POLL_SEC) == 0:
            print(f'  ...still waiting ({i * POLL_SEC:.0f}s elapsed)', file=sys.stderr, flush=True)
        try:
            tf = buf.lookup_transform('map', 'base_link', Time())
            break
        except Exception:
            pass

    if tf is None:
        print(f'no map -> base_link transform after {TIMEOUT_SEC:.0f}s '
              '- is AMCL up and converged (2D Pose Estimate set in RViz)?')
        rclpy.shutdown()
        sys.exit(1)

    t, q = tf.transform.translation, tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    print(f'x={t.x:.3f} y={t.y:.3f} phi_deg={math.degrees(yaw):.2f}', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
