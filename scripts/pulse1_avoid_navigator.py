#!/usr/bin/env python3
"""Navigatore AVOID: prima fase HOLD, senza movimento."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MAX_OBSERVER_AGE_S = 1.5

SENSOR_FLAGS = (
    "odom_ok",
    "lidar_ok",
    "rear_lidar_ok",
    "imu_ok",
)


def hold_request(observer, age_s):
    """Richiesta AVOID non motoria, sempre non autorizzabile."""

    fresh = (
        type(age_s) in (int, float)
        and math.isfinite(age_s)
        and 0 <= age_s <= MAX_OBSERVER_AGE_S
    )

    valid_observer = (
        fresh
        and isinstance(observer, dict)
        and observer.get("source") == "avoid_observer"
        and observer.get("mode") == "AVOID"
        and observer.get("state") == "SENSOR_READY"
        and observer.get("motor_commands") is False
    )

    flags = {
        name: (
            valid_observer
            and observer.get(name) is True
        )
        for name in SENSOR_FLAGS
    }

    ready = all(flags.values())

    return {
        "source": "avoid_navigator",
        "mode": "AVOID",
        "state": "HOLD",
        "reason": (
            "ZERO_MOTION_INTEGRATION"
            if ready else "WAIT_SENSORS"
        ),
        **flags,
        "proposed_linear_m_s": 0.0,
        "proposed_angular_rad_s": 0.0,
        "motor_commands": False,
    }


class AvoidNavigator(Node):

    def __init__(self):
        super().__init__("pulse1_avoid_navigator")

        self.observer = None
        self.observer_at = None

        self.subscription = self.create_subscription(
            String,
            "/pulse1/avoid_observer",
            self.on_observer,
            10,
        )

        self.output = self.create_publisher(
            String,
            "/pulse1/avoid_request",
            10,
        )

        self.create_timer(0.2, self.control)

        self.get_logger().info(
            "AVOID Navigator HOLD: movimento disabilitato"
        )

    def on_observer(self, msg):
        try:
            value = json.loads(msg.data)

            if not isinstance(value, dict):
                raise ValueError("Messaggio non valido")

            self.observer = value
            self.observer_at = time.monotonic()

        except (ValueError, TypeError):
            self.observer = None
            self.observer_at = None

    def control(self):
        age = (
            time.monotonic() - self.observer_at
            if self.observer_at is not None
            else None
        )

        result = hold_request(self.observer, age)

        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.output.publish(msg)


def main():
    rclpy.init(args=[])
    node = AvoidNavigator()

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
