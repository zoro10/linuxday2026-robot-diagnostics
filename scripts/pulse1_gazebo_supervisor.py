#!/usr/bin/env python3
"""Supervisore indipendente di emergenza per Gazebo S1."""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TIMEOUT_S = 2.0
STARTUP_GRACE_S = 12.0


def watchdog_state(start_at, last_at, now):
    if last_at is None:
        return (
            "WAITING"
            if now - start_at <= STARTUP_GRACE_S
            else "GUARD_LOST"
        )

    return (
        "HEALTHY"
        if 0 <= now - last_at <= TIMEOUT_S
        else "GUARD_LOST"
    )


class GazeboSupervisor(Node):

    def __init__(self, armed=False):
        super().__init__("pulse1_gazebo_supervisor")

        self.armed = armed
        self.started_at = time.monotonic()
        self.last_heartbeat = None
        self.last_state = None
        self.last_attempt = None
        self.pause_confirmed = False

        self.create_subscription(
            String,
            "/pulse1/motor_guard_live",
            self.on_heartbeat,
            10,
        )

        self.create_timer(0.5, self.check)

        self.get_logger().info(
            "Supervisore avviato: " +
            ("ARMED" if armed else "OBSERVE")
        )

    def on_heartbeat(self, msg):
        try:
            data = json.loads(msg.data)

            if (
                isinstance(data, dict)
                and data.get("source") == "motor_guard_live"
                and data.get("motor_commands") is True
                and data.get("guard_state") in (
                    "AUTHORIZED",
                    "BLOCKED",
                    "STALE_REQUEST",
                )
            ):
                self.last_heartbeat = time.monotonic()

        except (ValueError, TypeError):
            pass

    def check(self):
        now = time.monotonic()

        if self.pause_confirmed:
            return

        state = watchdog_state(
            self.started_at,
            self.last_heartbeat,
            now,
        )

        if state != self.last_state:
            self.get_logger().warning(
                f"Watchdog: {state}"
            )
            self.last_state = state

        if state != "GUARD_LOST" or not self.armed:
            return

        if (
            self.last_attempt is not None
            and now - self.last_attempt < 2.0
        ):
            return

        self.last_attempt = now

        command = [
            "gz", "service",
            "-s", "/world/empty/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", "pause: true",
        ]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=5,
            )

            output = result.stdout + result.stderr

            if (
                result.returncode == 0
                and "data: true" in output
            ):
                self.pause_confirmed = True
                self.get_logger().error(
                    "EMERGENZA: pausa Gazebo confermata"
                )
            else:
                self.get_logger().error(
                    "Pausa non confermata: " + output.strip()
                )

        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(
                f"Errore servizio emergenza: {exc}"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--armed", action="store_true")
    args = parser.parse_args()

    if args.armed:
        if os.environ.get("PULSE1_S1_SUPERVISOR_ARM") != "YES":
            raise SystemExit(
                "Supervisore disarmato: autorizzazione assente"
            )

        metadata = (
            Path(__file__).resolve().parent.parent
            / "runtime/pulse1_world.json"
        )

        try:
            scenario = json.loads(
                metadata.read_text()
            ).get("scenario")
        except (OSError, ValueError):
            scenario = None

        if scenario != "s1":
            raise SystemExit(
                "Supervisore armato consentito solo in S1"
            )

    rclpy.init(args=[])
    node = GazeboSupervisor(armed=args.armed)

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
