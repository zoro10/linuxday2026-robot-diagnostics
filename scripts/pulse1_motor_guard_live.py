#!/usr/bin/env python3
"""Motor Guard operativo per Demo 2 STOP."""

import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist

from pulse1_motor_guard_logic import (
    MAX_REQUEST_AGE_S,
    guarded_motion,
    authorized_request,
)


class MotorGuardLive(Node):

    def __init__(self, selected_mode='STOP'):
        super().__init__("pulse1_motor_guard_live")

        self.request_at = None
        self.request_speed = None
        self.request_angular = 0.0
        self.request_allowed = False
        self.upstream_state = None
        self.selected_mode = selected_mode
        self.sensor_at = {}

        self.create_subscription(
            String,
            ("/pulse1/stop_dryrun"
             if selected_mode == "STOP"
             else "/pulse1/avoid_request"),
            self.on_request,
            10,
        )

        if selected_mode == "AVOID":
            sensors = (
                (Odometry, "/model/pulse1_proto/odometry", "odom"),
                (LaserScan, "/pulse1/scan", "front"),
                (LaserScan, "/pulse1/scan_rear", "rear"),
                (Imu, "/pulse1/imu", "imu"),
            )

            for msg_type, topic, name in sensors:
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, n=name: self.on_sensor(n, msg),
                    qos_profile_sensor_data,
                )

        self.output = self.create_publisher(
            String,
            "/pulse1/motor_guard_live",
            10,
        )

        topic = "/model/pulse1_proto/cmd_vel"

        if self.get_publishers_info_by_topic(topic):
            raise RuntimeError(
                "cmd_vel ha gia un publisher: avvio negato"
            )

        self.motor = self.create_publisher(
            Twist, topic, 10
        )

        self.create_timer(0.2, self.control)

    def on_sensor(self, name, msg):
        if name == "odom":
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            values = (p.x, p.y, q.x, q.y, q.z, q.w)
            valid = (
                all(math.isfinite(v) for v in values)
                and sum(v*v for v in (q.x, q.y, q.z, q.w)) >= 0.5
            )
        elif name == "imu":
            a = msg.linear_acceleration
            g = msg.angular_velocity
            valid = all(
                math.isfinite(v)
                for v in (a.x, a.y, a.z, g.x, g.y, g.z)
            )
        else:
            valid = (
                len(msg.ranges) >= 5
                and math.isfinite(msg.angle_min)
                and math.isfinite(msg.angle_increment)
                and msg.angle_increment > 0
                and math.isfinite(msg.range_min)
                and math.isfinite(msg.range_max)
                and 0 < msg.range_min < msg.range_max
                and all(
                    value == math.inf or (
                        math.isfinite(value)
                        and msg.range_min <= value <= msg.range_max
                    )
                    for value in msg.ranges
                )
            )

        if valid:
            self.sensor_at[name] = time.monotonic()
        else:
            # Non riutilizzare una precedente lettura valida.
            self.sensor_at.pop(name, None)

    def on_request(self, msg):
        self.request_at = time.monotonic()
        self.request_speed = None
        self.request_angular = 0.0
        self.request_allowed = False
        self.upstream_state = "INVALID"

        try:
            data = json.loads(msg.data)

            if not isinstance(data, dict):
                return

            self.request_speed = data.get(
                "proposed_linear_m_s"
            )
            self.upstream_state = data.get("state")
            self.request_angular = (
                data.get("proposed_angular_rad_s")
                if self.selected_mode == "AVOID" else 0.0
            )

            self.request_allowed = authorized_request(
                data, self.selected_mode
            )

        except (ValueError, TypeError):
            pass

    def control(self):
        age = (
            time.monotonic() - self.request_at
            if self.request_at is not None
            else None
        )

        sensor_limits = {
            "odom": 2.5,
            "front": 1.5,
            "rear": 2.5,
            "imu": 1.0,
        }

        sensor_ready = (
            self.selected_mode == "STOP"
            or all(
                name in self.sensor_at
                and 0 <= time.monotonic() - self.sensor_at[name] <= limit
                for name, limit in sensor_limits.items()
            )
        )

        active = self.request_allowed and sensor_ready

        speed, angular = guarded_motion(
            self.request_speed,
            self.request_angular,
            age,
            active=active,
        )

        if age is None or age > MAX_REQUEST_AGE_S:
            guard_state = "STALE_REQUEST"
        elif not active:
            guard_state = "BLOCKED"
        else:
            guard_state = "AUTHORIZED"

        result = {
            "source": "motor_guard_live",
            "mode": self.selected_mode,
            "avoid_sensor_ready": sensor_ready,
            "upstream_state": self.upstream_state,
            "guard_state": guard_state,
            "request_age_s": (
                round(age, 3) if age is not None else None
            ),
            "permitted_linear_m_s": speed,
            "permitted_angular_rad_s": angular,
            "motor_commands": True,
        }

        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.output.publish(msg)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angular
        self.motor.publish(cmd)


def main():
    if os.environ.get("PULSE1_S1_MOTOR_ARM") != "YES":
        raise SystemExit(
            "Motor Guard disarmato: avvio motori non consentito"
        )

    selected_mode = os.environ.get("PULSE1_NAV_MODE", "STOP")

    if selected_mode not in ("STOP", "AVOID"):
        raise SystemExit("Modalita' motore non valida")

    if selected_mode == "AVOID":
        if os.environ.get("PULSE1_AVOID_MOTOR_ARM") != "YES":
            raise SystemExit("AVOID non armato")

        metadata = (
            Path(__file__).resolve().parent.parent
            / "runtime/pulse1_world.json"
        )

        try:
            world = json.loads(metadata.read_text())
        except (OSError, ValueError):
            world = {}

        if (
            world.get("scenario") != "s1"
            or world.get("navigation_mode") != "AVOID"
        ):
            raise SystemExit(
                "AVOID negato: scenario o modalita' non coerenti"
            )

    rclpy.init(args=[])
    node = MotorGuardLive(selected_mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.motor.publish(Twist())
                time.sleep(0.1)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
