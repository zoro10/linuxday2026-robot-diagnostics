#!/usr/bin/env python3
"""ROS2 waypoint *shadow*: publishes diagnostics, NEVER /avoid_request or cmd_vel."""

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pulse1_waypoint_plan import WaypointShadow, build_route, load_world


class WaypointShadowNode(Node):
    def __init__(self, side, world_path):
        super().__init__('pulse1_waypoint_shadow')
        route = build_route(load_world(world_path), side)
        self.shadow = WaypointShadow(route)
        self.observer = None
        self.observer_at = None
        self.sub_observer = self.create_subscription(
            String, '/pulse1/avoid_observer', self.on_observer, 10,
        )
        self.output = self.create_publisher(String, '/pulse1/waypoint_shadow', 10)
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info('WAYPOINT SHADOW S1: nessun comando motore')

    def on_observer(self, message):
        try:
            data = json.loads(message.data)
            self.observer = data if isinstance(data, dict) else None
        except (ValueError, TypeError):
            self.observer = None
        self.observer_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        age = now - self.observer_at if self.observer_at is not None else None
        result = self.shadow.evaluate(self.observer, age)
        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.output.publish(msg)


def main():
    parser = argparse.ArgumentParser(description='PULSE-1 S1 shadow-only waypoint navigator')
    parser.add_argument('--side', choices=('LEFT', 'RIGHT'), default='LEFT')
    parser.add_argument('--world', type=Path, default=Path(__file__).resolve().parent.parent / 'runtime/pulse1_world.json')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = WaypointShadowNode(args.side, args.world)
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
