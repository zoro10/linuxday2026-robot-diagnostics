#!/usr/bin/env python3
"""Controller STOP in prova: nessuna pubblicazione motore."""

import json
import math
import time

import rclpy
from std_msgs.msg import String

from pulse1_navigation_shadow import NavigationShadow
from pulse1_stop_policy import stop_velocity


class StopDryRun(NavigationShadow):

    def __init__(self):
        super().__init__(
            "STOP",
            node_name="pulse1_stop_dryrun",
        )

        # Sostituisce il timer shadow da 1 Hz
        # con il controllo da 5 Hz.
        self.timer.cancel()

        self.output = self.create_publisher(
            String, "/pulse1/stop_dryrun", 10
        )

        self.control_timer = self.create_timer(
            0.2, self.control
        )

        self.previous_state = None

    def control(self):
        now = time.monotonic()

        odom_age = (
            now - self.odom_at
            if self.odom_at is not None else None
        )
        lidar_age = (
            now - self.lidar_at
            if self.lidar_at is not None else None
        )

        odom_ok = (
            odom_age is not None
            and 0 <= odom_age <= 2.5
        )
        lidar_ok = (
            lidar_age is not None
            and 0 <= lidar_age <= 1.0
        )

        if self.odom is None:
            x = y = yaw = math.nan
        else:
            x, y, yaw = self.odom

        decision = self.fsm.update(
            x, y, yaw, self.front,
            odom_ok=odom_ok,
            lidar_ok=lidar_ok,
        )

        speed = stop_velocity(
            decision,
            self.front,
            odom_age,
            lidar_age,
            mode="STOP",
        )

        result = {
            "source": "stop_dryrun",
            "mode": "STOP",
            "state": decision.state,
            "reason": decision.reason,
            "goal_distance_m": decision.goal_distance_m,
            "front_m": (
                self.front
                if lidar_ok
                and self.front is not None
                and math.isfinite(self.front)
                else None
            ),
            "front_clear": bool(
                lidar_ok and self.front == math.inf
            ),
            "odom_ok": odom_ok,
            "lidar_ok": lidar_ok,
            "proposed_linear_m_s": speed,
            "motor_commands": False,
        }

        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.output.publish(msg)

        if decision.state != self.previous_state:
            self.get_logger().info(
                f"{decision.state} | "
                f"velocita proposta={speed:.2f} m/s"
            )
            self.previous_state = decision.state


def main():
    rclpy.init(args=[])
    node = StopDryRun()

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
