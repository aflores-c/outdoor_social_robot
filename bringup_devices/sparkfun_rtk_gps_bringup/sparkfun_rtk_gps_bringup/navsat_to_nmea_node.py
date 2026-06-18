#!/usr/bin/env python3
"""
Converts sensor_msgs/NavSatFix to nmea_msgs/Sentence (GPGGA) so the ntrip_client
can forward the rover position to the NTRIP caster — required for VRS mountpoints.
"""
import math
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence


class NavSatToNmea(Node):
    def __init__(self):
        super().__init__('navsat_to_nmea')
        self.sub = self.create_subscription(NavSatFix, '/fix', self._fix_cb, 10)
        self.pub = self.create_publisher(Sentence, '/nmea', 10)

    def _fix_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return  # no fix, skip

        now = datetime.now(timezone.utc)
        time_str = now.strftime('%H%M%S.00')

        lat, lon = msg.latitude, msg.longitude
        lat_d, lat_m = int(abs(lat)), (abs(lat) % 1) * 60.0
        lon_d, lon_m = int(abs(lon)), (abs(lon) % 1) * 60.0
        lat_hem = 'N' if lat >= 0 else 'S'
        lon_hem = 'E' if lon >= 0 else 'W'

        # sensor_msgs/NavSatStatus: 0=fix, 1=sbas, 2=gbas(RTK/DGPS)
        fix_q = {-1: 0, 0: 1, 1: 2, 2: 4}.get(msg.status.status, 1)

        alt = msg.altitude if not math.isnan(msg.altitude) else 0.0

        body = (
            f'GPGGA,{time_str},'
            f'{lat_d:02d}{lat_m:09.6f},{lat_hem},'
            f'{lon_d:03d}{lon_m:09.6f},{lon_hem},'
            f'{fix_q},00,1.0,{alt:.1f},M,0.0,M,,'
        )
        checksum = 0
        for c in body:
            checksum ^= ord(c)

        sentence = Sentence()
        sentence.header.stamp = self.get_clock().now().to_msg()
        sentence.header.frame_id = 'gps'
        sentence.sentence = f'${body}*{checksum:02X}'
        self.pub.publish(sentence)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NavSatToNmea())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
