#!/usr/bin/env python3
"""Acquisizione sensori AVOID. Nessun comando motore."""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from pulse1_scan_logic import front_distance
from pulse1_avoid_sectors import sector_distances
from pulse1_avoid_choice import choose_avoid_side


MAX_AGES = {
    "odom": 2.5,
    "front": 1.5,
    "rear": 2.5,
    "imu": 1.0,
}


class AvoidObserver(Node):

    def __init__(self):
        super().__init__("pulse1_avoid_observer")

        self.received_at = {}
        self.readings = {}
        self.front_sectors = None
        self.count = 0

        self.create_subscription(
            Odometry,
            "/model/pulse1_proto/odometry",
            self.on_odom,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            LaserScan,
            "/pulse1/scan",
            lambda msg: self.on_scan("front", msg),
            qos_profile_sensor_data,
        )

        self.create_subscription(
            LaserScan,
            "/pulse1/scan_rear",
            lambda msg: self.on_scan("rear", msg),
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Imu,
            "/pulse1/imu",
            self.on_imu,
            qos_profile_sensor_data,
        )

        self.output = self.create_publisher(
            String, "/pulse1/avoid_observer", 10
        )

        self.create_timer(0.2, self.report)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        values = (
            p.x, p.y, q.x, q.y, q.z, q.w
        )

        if not all(math.isfinite(x) for x in values):
            return

        norm = sum(x * x for x in (q.x, q.y, q.z, q.w))
        if norm < 0.5:
            return

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.readings["odom"] = {
            "x": round(p.x, 3),
            "y": round(p.y, 3),
            "yaw_rad": round(yaw, 4),
        }

        self.received_at["odom"] = time.monotonic()

    def on_scan(self, name, msg):
        if (
            not msg.ranges
            or not math.isfinite(msg.angle_increment)
            or msg.angle_increment == 0
            or not math.isfinite(msg.angle_min)
            or not math.isfinite(msg.range_min)
            or not math.isfinite(msg.range_max)
            or msg.range_min >= msg.range_max
        ):
            return

        distance = front_distance(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

        if (
            distance is None
            or not isinstance(distance, (int, float))
            or math.isnan(distance)
        ):
            return

        sectors = sector_distances(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

        # Nessun settore sconosciuto puo' essere
        # considerato sicuro per AVOID.
        if any(value is None for value in sectors.values()):
            return

        self.readings[name] = {
            "distance_m": (
                round(distance, 3)
                if math.isfinite(distance)
                else None
            ),
            "clear": distance == math.inf,
            "sectors": {
                sector: {
                    "distance_m": (
                        round(value, 3)
                        if math.isfinite(value)
                        else None
                    ),
                    "clear": value == math.inf,
                }
                for sector, value in sectors.items()
            },
        }

        if name == "front":
            self.front_sectors = dict(sectors)

        self.received_at[name] = time.monotonic()

    def on_imu(self, msg):
        a = msg.linear_acceleration
        g = msg.angular_velocity

        values = (a.x, a.y, a.z, g.x, g.y, g.z)

        if not all(math.isfinite(x) for x in values):
            return

        self.readings["imu"] = {
            "accel_m_s2": [
                round(a.x, 3),
                round(a.y, 3),
                round(a.z, 3),
            ],
            "gyro_rad_s": [
                round(g.x, 4),
                round(g.y, 4),
                round(g.z, 4),
            ],
        }

        self.received_at["imu"] = time.monotonic()

    def report(self):
        now = time.monotonic()
        ages = {}
        healthy = {}

        for name, limit in MAX_AGES.items():
            timestamp = self.received_at.get(name)

            age = (
                now - timestamp
                if timestamp is not None else None
            )

            healthy[name] = (
                age is not None
                and 0 <= age <= limit
            )

            ages[name] = (
                round(age, 2) if age is not None else None
            )

        all_ready = all(healthy.values())

        # Solo una valutazione: nessun comando motore.
        choice = (
            choose_avoid_side(self.front_sectors)
            if all_ready else 'WAIT_DATA'
        )

        result = {
            "source": "avoid_observer",
            "avoid_candidate": choice,
            "mode": "AVOID",
            "state": (
                "SENSOR_READY"
                if all_ready else "WAIT_SENSORS"
            ),
            "odom_ok": healthy["odom"],
            "lidar_ok": healthy["front"],
            "rear_lidar_ok": healthy["rear"],
            "imu_ok": healthy["imu"],
            "ages_s": ages,
            "readings": {
                name: (
                    self.readings.get(name)
                    if healthy[name] else None
                )
                for name in MAX_AGES
            },
            "proposed_linear_m_s": 0.0,
            "proposed_angular_rad_s": 0.0,
            "motor_commands": False,
        }

        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.output.publish(msg)

        print(msg.data, flush=True)
        self.count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Pubblica continuamente la percezione AVOID",
    )
    args = parser.parse_args()

    rclpy.init(args=[])
    node = AvoidObserver()

    try:
        if args.continuous:
            rclpy.spin(node)
        else:
            deadline = time.monotonic() + 12

            while node.count < 4 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)

            if node.count < 4:
                raise SystemExit("Timeout acquisizione observer")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
