#!/usr/bin/env python3

import serial

import rclpy
import rclpy.time
from rclpy.node import Node
from sensor_msgs.msg import Imu


class BNO055SerialImuNode(Node):
    def __init__(self):
        super().__init__("bno055_serial_imu_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic_name", "/imu/data")
        self.declare_parameter("debug_serial", False)

        self.declare_parameter("accel_scale", 1.042)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.topic_name = self.get_parameter("topic_name").value
        self.debug_serial = bool(self.get_parameter("debug_serial").value)
        self.accel_scale = float(self.get_parameter("accel_scale").value)

        # Hardcoded gyro bias correction in rad/s.
        self.gyro_bias_x = -0.00732
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        self.pub = self.create_publisher(Imu, self.topic_name, 50)

        self.serial_port = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0.0
        )

        self.serial_port.reset_input_buffer()
        self.buffer = ""
        self._clock_offset_ns = None

        self.get_logger().info(f"Connected to {self.port} at {self.baudrate}")
        self.get_logger().info(f"Using accel_scale: {self.accel_scale}")
        self.get_logger().info(
            f"Using gyro bias correction: "
            f"x={self.gyro_bias_x}, y={self.gyro_bias_y}, z={self.gyro_bias_z}"
        )

        self.timer = self.create_timer(0.001, self.read_serial)

    def read_serial(self):
        try:
            data = self.serial_port.read(self.serial_port.in_waiting or 1)

            if not data:
                return

            self.buffer += data.decode("utf-8", errors="ignore")

            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                self.parse_line(line.strip())

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def parse_line(self, line):
        if not line:
            return

        values = line.split(",")

        if self.debug_serial:
            self.get_logger().info(f"RAW SERIAL: {line}")

        if len(values) != 11:
            return

        try:
            arduino_ms = int(float(values[0]))

            qx = float(values[1])
            qy = float(values[2])
            qz = float(values[3])
            qw = float(values[4])

            raw_gx = float(values[5])
            raw_gy = float(values[6])
            raw_gz = float(values[7])

            gx = raw_gx - self.gyro_bias_x
            gy = raw_gy - self.gyro_bias_y
            gz = raw_gz - self.gyro_bias_z

            ax = float(values[8]) * self.accel_scale
            ay = float(values[9]) * self.accel_scale
            az = float(values[10]) * self.accel_scale

        except ValueError:
            return

        arduino_ns = arduino_ms * 1_000_000
        ros_now_ns = self.get_clock().now().nanoseconds

        if self._clock_offset_ns is None:
            self._clock_offset_ns = ros_now_ns - arduino_ns
            self.get_logger().info("Clock offset initialized")

        stamped_ns = self._clock_offset_ns + arduino_ns

        msg = Imu()
        msg.header.stamp = rclpy.time.Time(nanoseconds=stamped_ns).to_msg()
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


def main(args=None):
    rclpy.init(args=args)
    node = BNO055SerialImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()