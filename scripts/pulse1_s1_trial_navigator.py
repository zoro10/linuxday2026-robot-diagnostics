#!/usr/bin/env python3
"""Primo APPROACH S1: una sola prova, disarmato di default."""

import argparse
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from pulse1_s1_approach_navigator import observer_healthy
from pulse1_s1_approach_request import SENSOR_FLAGS
from pulse1_s1_trial_gate import TrialGate


MAX_INPUT_AGE_S = 0.5
MAX_STARTUP_S = 10.0


class TrialNavigator(Node):

    def __init__(self, armed=False):
        super().__init__("pulse1_s1_trial_navigator")

        self.armed = armed
        self.gate = TrialGate()
        self.created_at = time.monotonic()
        self.started_at = None
        self.observer = None
        self.shadow = None
        self.observer_at = None
        self.shadow_at = None
        self.guard = None
        self.guard_at = None

        self.sub_observer = self.create_subscription(
            String, "/pulse1/avoid_observer",
            self.on_observer, 10,
        )
        self.sub_shadow = self.create_subscription(
            String, "/pulse1/s1_approach_shadow",
            self.on_shadow, 10,
        )

        self.sub_guard = self.create_subscription(
            String, '/pulse1/motor_guard_live',
            self.on_guard, 10,
        )

        self.diagnostic = self.create_publisher(
            String, "/pulse1/s1_trial_status", 10,
        )

        self.request_pub = None

        if armed:
            if self.get_publishers_info_by_topic(
                "/pulse1/avoid_request"
            ):
                raise RuntimeError(
                    "Navigatore già presente: avvio negato"
                )

            self.request_pub = self.create_publisher(
                String, "/pulse1/avoid_request", 10,
            )

        self.create_timer(0.1, self.control)

        self.get_logger().info(
            "S1 TRIAL: " + ("ARMED" if armed else "DISARMED")
        )

    def receive(self, msg):
        try:
            data = json.loads(msg.data)
            return data if isinstance(data, dict) else None
        except (ValueError, TypeError):
            return None

    def on_observer(self, msg):
        self.observer = self.receive(msg)
        self.observer_at = time.monotonic()

    def on_shadow(self, msg):
        self.shadow = self.receive(msg)
        self.shadow_at = time.monotonic()

    def on_guard(self, msg):
        self.guard = self.receive(msg)
        self.guard_at = time.monotonic()

    def fresh(self, stamp, now):
        return (
            stamp is not None
            and 0 <= now - stamp <= MAX_INPUT_AGE_S
        )

    def control(self):
        now = time.monotonic()

        observer_ok = (
            self.fresh(self.observer_at, now)
            and observer_healthy(self.observer)
        )

        shadow_ok = (
            self.fresh(self.shadow_at, now)
            and isinstance(self.shadow, dict)
            and self.shadow.get("source") == "s1_approach_shadow"
            and self.shadow.get("mode") == "AVOID"
            and self.shadow.get("state") == "HOLD"
            and self.shadow.get("sensors_ok") is True
            and self.shadow.get("motor_commands") is False
            and self.shadow.get("proposed_linear_m_s") == 0
            and self.shadow.get("proposed_angular_rad_s") == 0
        )

        pose = self.shadow.get("pose") if shadow_ok else None

        flags = {
            name: bool(
                observer_ok
                and self.observer.get(name) is True
            )
            for name in SENSOR_FLAGS
        }

        decision = {
            "state": (
                self.shadow.get("shadow_state")
                if shadow_ok else "HOLD"
            ),
            "reason": (
                self.shadow.get("shadow_reason")
                if shadow_ok else "INPUT_UNAVAILABLE"
            ),
            "shadow_linear_m_s": (
                self.shadow.get("shadow_linear_m_s")
                if shadow_ok else 0.0
            ),
            "proposed_linear_m_s": 0.0,
            "proposed_angular_rad_s": 0.0,
            "motor_commands": False,
        }

        guard_ready = (
            self.fresh(self.guard_at, now)
            and isinstance(self.guard, dict)
            and self.guard.get("source") == "motor_guard_live"
            and self.guard.get("mode") == "AVOID"
            and self.guard.get("guard_state") == "BLOCKED"
            and self.guard.get("upstream_state") == "HOLD"
            and self.guard.get("avoid_sensor_ready") is True
            and self.guard.get("motor_commands") is True
            and self.guard.get("permitted_linear_m_s") == 0
            and self.guard.get("permitted_angular_rad_s") == 0
        )

        ready = (
            observer_ok
            and shadow_ok
            and guard_ready
            and decision["state"] == "APPROACH"
        )

        if (
            self.armed
            and self.started_at is None
            and self.gate.terminal_reason is None
        ):
            if now - self.created_at >= MAX_STARTUP_S:
                self.gate.terminal_reason = "STARTUP_TIMEOUT"
            elif ready:
                self.started_at = now

        trial_running = (
            self.armed
            and self.started_at is not None
        )

        elapsed = (
            now - self.started_at
            if self.started_at is not None else 0.0
        )

        request = self.gate.step(
            decision, flags, pose, elapsed,
            armed=trial_running,
        )

        status = {
            "source": "s1_trial_navigator",
            "armed": self.armed,
            "sensors_ready": ready,
            "trial_phase": request["trial_phase"],
            "trial_reason": request["trial_reason"],
            "request_state": request["state"],
            "pose": pose,
            "motor_commands": False,
        }

        msg = String()
        msg.data = json.dumps(status, allow_nan=False)
        self.diagnostic.publish(msg)

        if self.request_pub is not None:
            output = String()
            output.data = json.dumps(request, allow_nan=False)
            self.request_pub.publish(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--armed", action="store_true")
    args = parser.parse_args()

    if args.armed:
        if os.environ.get("PULSE1_S1_TRIAL_ARM") != "YES":
            raise SystemExit("Trial non autorizzato")

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
            raise SystemExit("Scenario trial non valido")

    rclpy.init(args=[])
    try:
        node = TrialNavigator(armed=args.armed)
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.request_pub is not None:
            node.gate.terminal_reason = "NAVIGATOR_SHUTDOWN"

            for _ in range(5):
                if not rclpy.ok():
                    break
                request = node.gate.step(
                    {}, {}, None, 0.0, armed=False
                )
                msg = String()
                msg.data = json.dumps(request, allow_nan=False)
                try:
                    node.request_pub.publish(msg)
                except Exception:
                    if rclpy.ok():
                        raise
                    break
                time.sleep(0.1)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
