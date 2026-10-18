#!/usr/bin/env python3
"""Geofence ROS in sola osservazione. Nessun armamento."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from pulse1_s1_geofence_logic import Geofence


FLAGS = ("odom_ok", "lidar_ok", "rear_lidar_ok", "imu_ok")


def trial_drive(data, age):
    return (
        isinstance(data, dict)
        and type(age) in (int, float)
        and math.isfinite(age)
        and 0 <= age <= 0.5
        and data.get("source") == "avoid_navigator"
        and data.get("mode") == "AVOID"
        and data.get("state") == "GO_TO_GOAL"
        and data.get("trial_phase") == "DRIVE"
        and data.get("trial_reason") == "LIMITED_FORWARD_TRIAL"
        and data.get("motor_commands") is False
        and data.get("proposed_angular_rad_s") == 0
        and type(data.get("proposed_linear_m_s")) in (int, float)
        and 0 < data["proposed_linear_m_s"] <= 0.10
        and all(data.get(flag) is True for flag in FLAGS)
    )


class GeofenceObserver(Node):

    def __init__(self):
        super().__init__("pulse1_s1_geofence_watchdog")

        self.values = {}
        self.stamps = {}
        self.started_at = None
        self.geofence = Geofence()

        self.sub_nav = self.create_subscription(
            String, "/pulse1/avoid_request",
            lambda msg: self.on_json("nav", msg), 10
        )

        self.sub_guard = self.create_subscription(
            String, "/pulse1/motor_guard_live",
            lambda msg: self.on_json("guard", msg), 10
        )

        self.sub_odom = self.create_subscription(
            Odometry, "/model/pulse1_proto/odometry",
            self.on_odom, qos_profile_sensor_data
        )

        self.sub_cmd = self.create_subscription(
            Twist, "/model/pulse1_proto/cmd_vel",
            self.on_cmd, 10
        )

        self.output = self.create_publisher(
            String, "/pulse1/s1_geofence_watchdog", 10
        )

        self.create_timer(0.1, self.control)

        self.get_logger().info(
            "Geofence OBSERVE: nessuna pausa e nessun comando motore"
        )

    def store(self, name, value):
        self.values[name] = value
        self.stamps[name] = time.monotonic()

    def on_json(self, name, msg):
        try:
            value = json.loads(msg.data)
            self.store(
                name, value if isinstance(value, dict) else None
            )
        except (ValueError, TypeError):
            self.store(name, None)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w

        if (
            not all(math.isfinite(v) for v in values)
            or not 0.5 <= norm <= 1.5
        ):
            self.store("pose", None)
            return

        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        self.store("pose", (p.x, p.y, yaw))

    def on_cmd(self, msg):
        self.store("cmd", (msg.linear.x, msg.angular.z))

    def age(self, name, now):
        if name not in self.stamps:
            return None
        return now - self.stamps[name]

    def control(self):
        now = time.monotonic()

        active = trial_drive(
            self.values.get("nav"),
            self.age("nav", now)
        )

        if active and self.started_at is None:
            self.started_at = now

        elapsed = (
            now - self.started_at
            if self.started_at is not None else None
        )

        result = self.geofence.step(
            active=active,
            pose=self.values.get("pose"),
            pose_age_s=self.age("pose", now),
            guard=self.values.get("guard"),
            guard_age_s=self.age("guard", now),
            cmd=self.values.get("cmd"),
            cmd_age_s=self.age("cmd", now),
            elapsed_s=elapsed,
        )

        status = {
            "source": "s1_geofence_watchdog",
            "mode": "AVOID",
            "armed": False,
            "ready_for_motion": False,
            "trial_active": active,
            "action": result["action"],
            "reason": result["reason"],
            "motor_commands": False,
        }

        msg = String()
        msg.data = json.dumps(status, allow_nan=False)
        self.output.publish(msg)


def main():
    rclpy.init(args=[])
    node = GeofenceObserver()

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
