#!/usr/bin/env python3
"""Navigatore S1 APPROACH shadow: nessun comando motore."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from pulse1_s1_approach_logic import approach_step


SENSOR_FLAGS = (
    "odom_ok",
    "lidar_ok",
    "rear_lidar_ok",
    "imu_ok",
)


def observer_healthy(data):
    return (
        isinstance(data, dict)
        and data.get("source") == "avoid_observer"
        and data.get("mode") == "AVOID"
        and data.get("state") == "SENSOR_READY"
        and data.get("motor_commands") is False
        and all(data.get(k) is True for k in SENSOR_FLAGS)
    )


def front_distance(data):
    if not isinstance(data, dict):
        return None

    sector = (
        data.get("readings", {})
        .get("front", {})
        .get("sectors", {})
        .get("front", {})
    )

    if not isinstance(sector, dict):
        return None

    if sector.get("clear") is not False:
        return None

    distance = sector.get("distance_m")

    if (
        type(distance) not in (int, float)
        or not math.isfinite(distance)
        or distance <= 0
    ):
        return None

    return float(distance)


class ApproachShadow(Node):

    def __init__(self):
        super().__init__("pulse1_s1_approach_shadow")

        self.observer = None
        self.observer_at = None
        self.pose = None
        self.pose_at = None

        self.sub_observer = self.create_subscription(
            String,
            "/pulse1/avoid_observer",
            self.on_observer,
            10,
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            "/model/pulse1_proto/odometry",
            self.on_odom,
            qos_profile_sensor_data,
        )

        # SOLO diagnostica: NON è il topic Motor Guard.
        self.output = self.create_publisher(
            String,
            "/pulse1/s1_approach_shadow",
            10,
        )

        self.create_timer(0.2, self.control)

        self.get_logger().info(
            "S1 APPROACH SHADOW: motori sempre disabilitati"
        )

    def on_observer(self, msg):
        try:
            value = json.loads(msg.data)
            self.observer = (
                value if isinstance(value, dict) else None
            )
        except (ValueError, TypeError):
            self.observer = None

        self.observer_at = time.monotonic()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w

        if (
            not all(math.isfinite(v) for v in values)
            or not 0.5 <= norm <= 1.5
        ):
            self.pose = None
            self.pose_at = None
            return

        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z),
        )

        self.pose = (p.x, p.y, yaw)
        self.pose_at = time.monotonic()

    def control(self):
        now = time.monotonic()

        age = (
            now - self.observer_at
            if self.observer_at is not None
            else None
        )

        pose_fresh = (
            self.pose_at is not None
            and 0 <= now - self.pose_at <= 0.6
        )

        sensors_ok = (
            observer_healthy(self.observer)
            and age is not None
            and 0 <= age <= 1.5
            and pose_fresh
        )

        distance = (
            front_distance(self.observer)
            if sensors_ok else None
        )

        decision = approach_step(
            front_distance_m=distance,
            pose=self.pose if pose_fresh else None,
            sensors_ok=sensors_ok,
            observer_age_s=age,
        )

        diagnostic = {
            "source": "s1_approach_shadow",
            "mode": "AVOID",
            "state": "HOLD",
            "shadow_state": decision["state"],
            "shadow_reason": decision["reason"],
            "shadow_linear_m_s": decision[
                "shadow_linear_m_s"
            ],
            "front_distance_m": distance,
            "front_bumper_gap_m": decision[
                "front_bumper_gap_m"
            ],
            "pose": self.pose if pose_fresh else None,
            "sensors_ok": sensors_ok,
            "proposed_linear_m_s": 0.0,
            "proposed_angular_rad_s": 0.0,
            "motor_commands": False,
        }

        msg = String()
        msg.data = json.dumps(
            diagnostic, allow_nan=False
        )
        self.output.publish(msg)


def main():
    # Nessuna modalità armata prevista in questa versione.
    rclpy.init(args=[])

    node = ApproachShadow()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
