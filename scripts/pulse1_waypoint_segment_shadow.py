#!/usr/bin/env python3
"""Solo diagnostica ROS2: il nodo NON crea publisher su /avoid_request o /cmd_vel."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import WorldStatistics

from pulse1_waypoint_plan import build_route, load_world
from pulse1_waypoint_segment_contract import propose_segment


class WaypointSegmentShadowNode(Node):
    def __init__(self, side, world_path):
        super().__init__('pulse1_waypoint_segment_shadow')
        self.route = build_route(load_world(world_path), side)
        self.values = {}
        self.stamps = {}
        for name, topic in (
            ('waypoint', '/pulse1/waypoint_shadow'),
            ('observer', '/pulse1/avoid_observer'),
            ('guard', '/pulse1/v6_motor_guard'),
        ):
            self.create_subscription(String, topic, lambda m, n=name: self.on_json(n, m), 10)
        self.create_subscription(Twist, '/model/pulse1_proto/cmd_vel',
                                 self.on_cmd, 10)
        self.create_subscription(WorldStatistics, '/world/empty/stats',
                                 self.on_stats, qos_profile_sensor_data)
        self.publisher = self.create_publisher(String, '/pulse1/waypoint_segment_shadow', 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info('S1 WAYPOINT SEGMENT SHADOW: NO MOTOR PUBLISHERS')

    def on_json(self, name, msg):
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            value = None
        self.values[name] = value if isinstance(value, dict) else None
        self.stamps[name] = time.monotonic()

    def on_cmd(self, msg):
        self.values['cmd'] = (float(msg.linear.x), float(msg.angular.z))
        self.stamps['cmd'] = time.monotonic()

    def on_stats(self, msg):
        self.values['world_paused'] = msg.paused
        self.stamps['world_paused'] = time.monotonic()

    def tick(self):
        now = time.monotonic()

        def age(name):
            stamp = self.stamps.get(name)
            return now - stamp if stamp is not None else None

        status = propose_segment(
            route=self.route,
            waypoint=self.values.get('waypoint'), waypoint_age_s=age('waypoint'),
            observer=self.values.get('observer'), observer_age_s=age('observer'),
            guard=self.values.get('guard'), guard_age_s=age('guard'),
            cmd=self.values.get('cmd'), cmd_age_s=age('cmd'),
            world_paused=self.values.get('world_paused'), world_age_s=age('world_paused'),
        )
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description='PULSE-1 waypoint segment preview; NO MOTION')
    parser.add_argument('--side', choices=('LEFT', 'RIGHT'), default='LEFT')
    parser.add_argument('--world', type=Path,
                        default=Path(__file__).resolve().parent.parent / 'runtime/pulse1_world.json')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = WaypointSegmentShadowNode(args.side, args.world)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
