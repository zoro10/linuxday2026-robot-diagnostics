#!/usr/bin/env python3

import math
import time
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Pulse1Controller(Node):
    def __init__(self):
        super().__init__('pulse1_controller')

        self.cmd = self.create_publisher(
            Twist, '/model/pulse1_proto/cmd_vel', 10
        )

        self.create_subscription(
            LaserScan, '/pulse1/scan',
            self.on_scan, qos_profile_sensor_data
        )

        self.last_scan = None
        self.distance = float('inf')
        self.state = None

        self.create_timer(0.2, self.control)
        self.get_logger().info('PULSE-1 controller avviato')

    def on_scan(self, msg):
        self.last_scan = time.monotonic()

        front = [
            d for i, d in enumerate(msg.ranges)
            if abs(msg.angle_min + i * msg.angle_increment) < 0.35
            and math.isfinite(d)
            and msg.range_min <= d <= msg.range_max
        ]

        self.distance = min(front, default=float('inf'))

    def control(self):
        if self.last_scan is None or time.monotonic() - self.last_scan > 1.5:
            state = 'NO_LIDAR'
            speed = 0.0
        elif self.distance < 1.2:
            state = 'OSTACOLO'
            speed = 0.0
        else:
            state = 'AVANTI'
            speed = 0.18

        msg = Twist()
        msg.linear.x = speed
        self.cmd.publish(msg)

        if state != self.state:
            self.get_logger().info(
                f'{state} | distanza={self.distance:.2f} m | velocita={speed:.2f}'
            )
            self.state = state


def main():
    rclpy.init()
    node = Pulse1Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
            time.sleep(0.5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
