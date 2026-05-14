#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class BNO055SerialImuNode(Node):
    def __init__(self):
        super().__init__("bno055_serial_imu_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic_name", "/imu/data")

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.topic_name = self.get_parameter("topic_name").value

        self.pub = self.create_publisher(Imu, self.topic_name, 20)

        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.02
            )
            self.get_logger().info(f"Connected to {self.port} at {self.baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise e

        self.timer = self.create_timer(0.001, self.read_serial)

    def read_serial(self):
        try:
            line = self.serial_port.readline().decode("utf-8").strip()

            if not line:
                return

            if line.startswith("time") or line.startswith("Format"):
                return

            values = line.split(",")

            if len(values) != 11:
                self.get_logger().warn(f"Bad line: {line}")
                return

            time_ms = float(values[0])

            qx = float(values[1])
            qy = float(values[2])
            qz = float(values[3])
            qw = float(values[4])

            gx = float(values[5])
            gy = float(values[6])
            gz = float(values[7])

            ax = float(values[8])
            ay = float(values[9])
            az = float(values[10])

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.05
            ]

            msg.angular_velocity_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001
            ]

            msg.linear_acceleration_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.05
            ]

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Serial parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BNO055SerialImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()