#!/usr/bin/env python3
import json
import math
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from math import atan2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import WorldStatistics
from pulse1_linux_metrics import LinuxMetrics
from pulse1_health_score import evaluate
from pulse1_health_v2 import evaluate as evaluate_v2
from pulse1_web_check import check_web
from pulse1_events import EventJournal


class Pulse1Collector(Node):
    def __init__(self):
        super().__init__('pulse1_collector')

        self.started = time.monotonic()
        self.output = (
            Path(__file__).resolve().parent.parent
            / 'runtime' / 'pulse1_status.json'
        )

        self.events = EventJournal(
            self.output.with_name('pulse1_events.jsonl')
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
        self.sim_stats_at = None
        self.sim_paused = None
        # Read-only V6 safety telemetry. No command publishers or rearm paths.
        self.safety_values = {}
        self.safety_seen = {}

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

        self.create_subscription(
            WorldStatistics,
            '/world/empty/stats',
            self.on_world_stats,
            qos_profile_sensor_data
        )

        for key, topic in (
            ('guard', '/pulse1/v6_motor_guard'),
            ('geofence', '/pulse1/v6_geofence_watchdog'),
            ('supervisor', '/pulse1/v6_supervisor_status'),
        ):
            self.create_subscription(
                String, topic,
                lambda msg, name=key: self.on_safety(name, msg), 10
            )
        self.create_subscription(
            Twist, '/model/pulse1_proto/cmd_vel',
            self.on_cmd, 10
        )

        self.create_timer(1.0, self.write_status)
        self.get_logger().info('PULSE-1 collector avviato')

    def on_world_stats(self, msg):
        self.sim_paused = bool(msg.paused)
        self.sim_stats_at = time.monotonic()

    def on_safety(self, name, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(value, dict):
            self.safety_values[name] = value
            self.safety_seen[name] = time.monotonic()

    def on_cmd(self, msg):
        values = (msg.linear.x, msg.angular.z)
        if all(math.isfinite(v) for v in values):
            self.safety_values['cmd'] = list(values)
            self.safety_seen['cmd'] = time.monotonic()

    def mark(self, name):
        now = time.monotonic()
        self.topics[name]['last'] = now
        self.topics[name]['times'].append(now)

    def on_odom(self, msg):
        self.mark('odometry')
        q = msg.pose.pose.orientation
        yaw = atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        self.position = {
            'x': round(msg.pose.pose.position.x, 3),
            'y': round(msg.pose.pose.position.y, 3),
            'speed_m_s': round(msg.twist.twist.linear.x, 3),
            'yaw_rad': round(yaw, 4)
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

        stats_age = (
            now - self.sim_stats_at
            if self.sim_stats_at is not None else None
        )

        if (stats_age is None or
                stats_age < 0 or stats_age > 3.0):
            simulation_state = 'UNKNOWN'
        elif self.sim_paused:
            simulation_state = 'PAUSED'
        else:
            simulation_state = 'RUNNING'
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

        try:
            world = json.loads(
                self.output.with_name('pulse1_world.json').read_text()
            )
        except (OSError, ValueError):
            world = {}

        navigation_mode = (
            world.get('navigation_mode')
            if world.get('scenario') == 's1'
            else None
        )

        v6_nodes = ("pulse1_v6_motor_guard", "pulse1_v6_geofence_watchdog", "pulse1_v6_supervisor")
        v6_active = any(name in discovered for name in v6_nodes)
        expected_controller = "pulse1_v6_motor_guard" if v6_active else "pulse1_motor_guard_live"

        nodes = {
            'pulse1_bridge': 'pulse1_bridge' in discovered,
            expected_controller: expected_controller in discovered
        }

        if v6_active:
            nodes.update({name: name in discovered for name in v6_nodes})

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

        # Safety status must be fresh independently of collector heartbeat.
        safety_fresh = {
            key: (key in self.safety_seen
                  and 0 <= now - self.safety_seen[key] <= 2.0)
            for key in ('guard', 'geofence', 'supervisor', 'cmd')
        }
        safety = {
            'fresh': safety_fresh,
            **{key: self.safety_values.get(key)
               for key in ('guard', 'geofence', 'supervisor', 'cmd')},
        }

        payload = {
            "linux": self.linux_metrics.collect(),
            'robot': 'PULSE-1',
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'nodes': nodes,
            'controller_expected': expected_controller,
            'navigation_mode': navigation_mode,
            'simulation': {
                'state': simulation_state,
                'age_seconds': (
                    None if stats_age is None
                    else round(stats_age, 2)
                )
            },
            'topics': result,
            'safety': safety,
            'position': self.position,
            'lidar_front_distance_m': self.front_distance
        }

        if now - self.web_last_check >= 5.0:
            self.web_state = check_web(timeout=2)
            self.web_last_check = now
        payload['web'] = self.web_state
        payload['health'] = evaluate(payload)
        payload['diagnostics'] = evaluate_v2(payload)

        if now - self.started >= 8.0:
            for event in self.events.update(nodes, result):
                self.get_logger().warning(
                    f"{event['type']}: {event['component']}"
                )

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
