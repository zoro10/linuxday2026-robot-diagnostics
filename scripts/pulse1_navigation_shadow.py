#!/usr/bin/env python3
"""Navigatore ROS 2 in sola osservazione: nessun comando motore."""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from pulse1_navigation_fsm import MissionFSM


class NavigationShadow(Node):

    def __init__(self, mode):
        super().__init__("pulse1_navigation_shadow")

        # PULSE-1 viene inserito in S1 in (0, 0, 0).
        # La meta rimane X=10, Y=0 anche se il nodo parte
        # quando il rover si è già spostato.
        self.fsm = MissionFSM(mode, 0.0, 0.0, 0.0)
        self.odom = None
        self.front = None
        self.odom_at = None
        self.lidar_at = None
        self.count = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            "/model/pulse1_proto/odometry",
            self.on_odom,
            qos_profile_sensor_data,
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/pulse1/scan",
            self.on_scan,
            qos_profile_sensor_data,
        )

        self.decision_pub = self.create_publisher(
            String, "/pulse1/decision_shadow", 10
        )

        self.timer = self.create_timer(1.0, self.evaluate)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.odom = (p.x, p.y, yaw)
        self.odom_at = time.monotonic()

    def on_scan(self, msg):
        values = [
            d
            for i, d in enumerate(msg.ranges)
            if abs(msg.angle_min + i * msg.angle_increment) < 0.35
            and math.isfinite(d)
            and msg.range_min <= d <= msg.range_max
        ]

        self.front = min(values) if values else None
        self.lidar_at = time.monotonic()

    def evaluate(self):
        now = time.monotonic()

        odom_age = (
            now - self.odom_at
            if self.odom_at is not None else None
        )
        lidar_age = (
            now - self.lidar_at
            if self.lidar_at is not None else None
        )

        odom_ok = odom_age is not None and odom_age <= 3.0
        lidar_ok = lidar_age is not None and lidar_age <= 1.5

        if self.odom is None:
            x = y = yaw = float("nan")
        else:
            x, y, yaw = self.odom

        decision = self.fsm.update(
            x, y, yaw, self.front,
            odom_ok=odom_ok,
            lidar_ok=lidar_ok,
        )

        result = {
            "source": "navigation_shadow",
            "mode": self.fsm.mode,
            "state": decision.state,
            "reason": decision.reason,
            "goal_x": self.fsm.goal_x,
            "goal_y": self.fsm.goal_y,
            "goal_distance_m": decision.goal_distance_m,
            "heading_error_rad": decision.heading_error_rad,
            "front_m": self.front if lidar_ok else None,
            "odom_age_s": round(odom_age, 2)
                if odom_age is not None else None,
            "lidar_age_s": round(lidar_age, 2)
                if lidar_age is not None else None,
            "motor_commands": False,
        }

        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.decision_pub.publish(msg)

        print(msg.data, flush=True)
        self.count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode", choices=["STOP", "AVOID"], required=True
    )
    parser.add_argument("--samples", type=int, default=6)
    args = parser.parse_args()

    if args.samples < 0:
        parser.error("--samples deve essere >= 0 (0 = continuo)")

    rclpy.init(args=[])
    node = NavigationShadow(args.mode)

    try:
        if args.samples == 0:
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            return

        deadline = time.monotonic() + max(15, args.samples * 3)

        while (
            node.count < args.samples
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.2)

        if node.count < args.samples:
            raise SystemExit("Timeout: decisioni insufficienti")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
