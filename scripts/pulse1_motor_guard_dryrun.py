#!/usr/bin/env python3
"""Motor Guard ROS 2 in osservazione: nessun Twist."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pulse1_motor_guard_logic import (
    MAX_REQUEST_AGE_S,
    guarded_velocity,
)


class MotorGuardDryRun(Node):

    def __init__(self):
        super().__init__("pulse1_motor_guard_dryrun")

        self.request_at = None
        self.request_speed = None
        self.request_allowed = False
        self.upstream_state = None

        self.create_subscription(
            String,
            "/pulse1/stop_dryrun",
            self.on_request,
            10,
        )

        self.output = self.create_publisher(
            String,
            "/pulse1/motor_guard_dryrun",
            10,
        )

        self.create_timer(0.2, self.control)

    def on_request(self, msg):
        self.request_at = time.monotonic()
        self.request_speed = None
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

            self.request_allowed = all((
                data.get("source") == "stop_dryrun",
                data.get("mode") == "STOP",
                data.get("state") == "DRIVE",
                data.get("odom_ok") is True,
                data.get("lidar_ok") is True,
                data.get("motor_commands") is False,
            ))

        except (ValueError, TypeError):
            pass

    def control(self):
        age = (
            time.monotonic() - self.request_at
            if self.request_at is not None
            else None
        )

        speed = guarded_velocity(
            self.request_speed,
            age,
            active=self.request_allowed,
        )

        if age is None or age > MAX_REQUEST_AGE_S:
            guard_state = "STALE_REQUEST"
        elif not self.request_allowed:
            guard_state = "BLOCKED"
        else:
            guard_state = "AUTHORIZED"

        result = {
            "source": "motor_guard_dryrun",
            "upstream_state": self.upstream_state,
            "guard_state": guard_state,
            "request_age_s": (
                round(age, 3) if age is not None else None
            ),
            "permitted_linear_m_s": speed,
            "motor_commands": False,
        }

        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.output.publish(msg)


def main():
    rclpy.init(args=[])
    node = MotorGuardDryRun()

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
