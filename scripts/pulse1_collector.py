#!/usr/bin/env python3
import json
import math
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from pulse1_linux_metrics import LinuxMetrics
from pulse1_health_score import evaluate
from pulse1_health_v2 import evaluate as evaluate_v2
from pulse1_web_check import check_web


class Pulse1Collector(Node):
    def __init__(self):
        super().__init__('pulse1_collector')

        self.started = time.monotonic()
        self.output = (
            Path(__file__).resolve().parent.parent
            / 'runtime' / 'pulse1_status.json'
        )

        self.topics = {
            'odometry': {'last': None, 'times': deque(maxlen=30), 'timeout': 3.5},
            'imu': {'last': None, 'times': deque(maxlen=30), 'timeout': 2.0},
            'lidar': {'last': None, 'times': deque(maxlen=30), 'timeout': 2.0},
        }

        self.position = None
        self.front_distance = None
        self.last_status = None
        self.linux_metrics = LinuxMetrics()
        self.web_last_check = 0.0
        self.web_state = {'status': 'NOT_CHECKED'}

        self.create_subscription(
            Odometry, '/model/pulse1_proto/odometry',
            self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, '/pulse1/imu',
            self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, '/pulse1/scan',
            self.on_scan, qos_profile_sensor_data
        )

        self.create_timer(1.0, self.write_status)
        self.get_logger().info('PULSE-1 collector avviato')

    def mark(self, name):
        now = time.monotonic()
        self.topics[name]['last'] = now
        self.topics[name]['times'].append(now)

    def on_odom(self, msg):
        self.mark('odometry')
        self.position = {
            'x': round(msg.pose.pose.position.x, 3),
            'y': round(msg.pose.pose.position.y, 3),
            'speed_m_s': round(msg.twist.twist.linear.x, 3)
        }

    def on_imu(self, msg):
        self.mark('imu')

    def on_scan(self, msg):
        self.mark('lidar')
        front = [
            distance
            for i, distance in enumerate(msg.ranges)
            if abs(msg.angle_min + i * msg.angle_increment) < 0.35
            and math.isfinite(distance)
            and msg.range_min <= distance <= msg.range_max
        ]
        self.front_distance = round(min(front), 2) if front else None

    def write_status(self):
        now = time.monotonic()
        result = {}

        for name, data in self.topics.items():
            last = data['last']
            age = None if last is None else now - last
            samples = data['times']

            rate = None
            if len(samples) >= 2 and samples[-1] > samples[0]:
                rate = (len(samples) - 1) / (samples[-1] - samples[0])

            if age is None:
                state = 'WAITING' if now - self.started < 6 else 'STALE'
            elif age > data['timeout']:
                state = 'STALE'
            else:
                state = 'OK'

            result[name] = {
                'status': state,
                'age_seconds': None if age is None else round(age, 2),
                'rate_hz': None if rate is None else round(rate, 2)
            }

        discovered = set(self.get_node_names())
        nodes = {
            'pulse1_bridge': 'pulse1_bridge' in discovered,
            'pulse1_controller': 'pulse1_controller' in discovered
        }

        healthy = (
            all(item['status'] == 'OK' for item in result.values())
            and all(nodes.values())
        )

        if healthy:
            status = 'OK'
        elif now - self.started < 6:
            status = 'STARTING'
        else:
            status = 'DEGRADED'

        payload = {
            "linux": self.linux_metrics.collect(),
            'robot': 'PULSE-1',
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'nodes': nodes,
            'topics': result,
            'position': self.position,
            'lidar_front_distance_m': self.front_distance
        }

        if now - self.web_last_check >= 5.0:
            self.web_state = check_web(timeout=2)
            self.web_last_check = now
        payload['web'] = self.web_state
        payload['health'] = evaluate(payload)
        payload['diagnostics'] = evaluate_v2(payload)

        temporary = self.output.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(payload, indent=2) + '\n')
        temporary.replace(self.output)

        if status != self.last_status:
            self.get_logger().info(f'Stato diagnostico: {status}')
            self.last_status = status


def main():
    rclpy.init()
    node = Pulse1Collector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
