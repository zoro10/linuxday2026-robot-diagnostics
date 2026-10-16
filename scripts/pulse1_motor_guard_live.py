#!/usr/bin/env python3
"""Motor Guard operativo per Demo 2 STOP."""

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from pulse1_motor_guard_logic import (
    MAX_REQUEST_AGE_S,
    guarded_motion,
    authorized_request,
)


class MotorGuardLive(Node):

    def __init__(self):
        super().__init__("pulse1_motor_guard_live")

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

            self.request_allowed = authorized_request(
                data, "STOP"
            )

        except (ValueError, TypeError):
            pass

    def control(self):
        age = (
            time.monotonic() - self.request_at
            if self.request_at is not None
            else None
        )

        speed, angular = guarded_motion(
            self.request_speed,
            0.0,  # Modalità STOP: nessuna rotazione
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
            "source": "motor_guard_live",
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

    rclpy.init(args=[])
    node = MotorGuardLive()

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
