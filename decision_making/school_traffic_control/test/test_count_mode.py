"""Standalone test for the VehicleDetectionCounts mode-smoothing logic
(_count_mode / _parking_space_free) — pure Python buffer trimming/mode
math, exercised directly rather than through ROS topic timing."""

from rclpy.duration import Duration
import pytest
import rclpy

from school_traffic_control.school_traffic_control_node import SchoolTrafficControlNode


@pytest.fixture
def node():
    rclpy.init()
    n = SchoolTrafficControlNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _seed(node, ages_and_counts):
    """ages_and_counts: list of (seconds_ago, raw_detections)."""
    now = node.get_clock().now()
    for seconds_ago, count in ages_and_counts:
        node._vehicle_counts_buffer.append((now - Duration(seconds=seconds_ago), count))


def test_mode_of_recent_samples(node):
    _seed(node, [(0.1, 10), (0.2, 10), (0.3, 15)])
    assert node._count_mode() == 10


def test_old_samples_trimmed_out(node):
    # 2.0s ago is outside the default 1.0s window and must not count.
    _seed(node, [(2.0, 30), (0.1, 8), (0.1, 8)])
    assert node._count_mode() == 8


def test_empty_buffer_is_none(node):
    assert node._count_mode() is None


def test_parking_space_free_below_threshold(node):
    _seed(node, [(0.1, 5), (0.1, 5), (0.1, 6)])
    assert node._parking_space_free() is True


def test_parking_space_not_free_at_or_above_threshold(node):
    _seed(node, [(0.1, 12), (0.1, 12), (0.1, 20)])
    assert node._parking_space_free() is False


def test_parking_space_not_free_when_no_samples(node):
    assert node._parking_space_free() is False
