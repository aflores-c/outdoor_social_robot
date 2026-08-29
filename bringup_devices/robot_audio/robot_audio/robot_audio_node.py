#!/usr/bin/env python3
"""
Robot audio playback node.

Exposes a PlayAudio action server: given a file name, plays the matching
mp3 file through the system's default audio output. Files are looked up
by name in this package's bundled audio/ directory (share/robot_audio/audio
once installed) — see the `audio_dir` parameter to point at a different
directory instead.
"""

import os
import time
import subprocess

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from robot_audio_msgs.action import PlayAudio


class RobotAudioNode(Node):

    def __init__(self):
        super().__init__('robot_audio_node')

        # Empty (default) -> this package's installed audio/ directory.
        self.declare_parameter('audio_dir', '')
        self.declare_parameter('play_audio_action', 'play_audio')
        # Any player that takes "<command> <file>" and plays it to the
        # default audio device works; ffplay handles mp3 natively.
        self.declare_parameter('player_command', 'ffplay')

        audio_dir = self.get_parameter('audio_dir').value
        if not audio_dir:
            audio_dir = os.path.join(get_package_share_directory('robot_audio'), 'audio')
        self._audio_dir = audio_dir

        self._player_command = self.get_parameter('player_command').value
        action_name = self.get_parameter('play_audio_action').value

        # ReentrantCallbackGroup + MultiThreadedExecutor so a blocking
        # playback doesn't stall goal acceptance/cancellation for other
        # goals (see main()).
        self._action_server = ActionServer(
            self,
            PlayAudio,
            action_name,
            execute_callback=self._execute_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"robot_audio_node ready — audio_dir='{self._audio_dir}', "
            f"action='{action_name}', player='{self._player_command}'")

    def _resolve_path(self, file_name: str) -> str:
        # basename strips any directory components a caller might sneak in
        # (e.g. "../../etc/passwd") — playback stays confined to audio_dir.
        name = os.path.basename(file_name)
        if not name.lower().endswith('.mp3'):
            name += '.mp3'
        return os.path.join(self._audio_dir, name)

    def _cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        file_name = goal_handle.request.file_name
        path = self._resolve_path(file_name)

        result = PlayAudio.Result()

        if not os.path.isfile(path):
            self.get_logger().warn(f"Audio file not found: {path}")
            result.success = False
            result.message = f"Audio file not found: {path}"
            goal_handle.abort()
            return result

        self.get_logger().info(f"Playing '{path}'")
        proc = subprocess.Popen(
            [self._player_command, '-nodisp', '-autoexit', '-loglevel', 'quiet', path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        while proc.poll() is None:
            if goal_handle.is_cancel_requested:
                proc.terminate()
                proc.wait()
                goal_handle.canceled()
                result.success = False
                result.message = 'Playback canceled'
                return result
            time.sleep(0.1)

        if proc.returncode == 0:
            result.success = True
            result.message = f"Played '{os.path.basename(path)}'"
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f"'{self._player_command}' exited with code {proc.returncode}"
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotAudioNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
