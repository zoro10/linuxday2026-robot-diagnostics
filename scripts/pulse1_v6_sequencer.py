#!/usr/bin/env python3
"""V6 one-segment orchestrator. Default: read-only PLAN."""
import argparse
import json
import math
import os
import re
import secrets
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import WorldStatistics

from pulse1_waypoint_plan import load_world
from pulse1_v6_route_policy import next_route_request

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "runtime/pulse1_world.json"
CMD = "/model/pulse1_proto/cmd_vel"
REQUEST = "/pulse1/v6_segment_request"
PREPARE = "/pulse1/v6_guard_rearm_prepare"
GEO_REARM = "/pulse1/v6_geofence_rearm"


def gazebo_pose():
    """Read-only Gazebo pose, with bounded retry and fail-closed."""
    last_error = None

    for attempt in range(1, 5):
        try:
            result = subprocess.run(
                ["gz", "model", "-m", "pulse1_proto", "-p"],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )

            match = re.search(
                r"Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*"
                r"\[([^\]]+)\]\s*\[([^\]]+)\]",
                result.stdout,
            )

            if (
                result.returncode == 0
                and "Name: pulse1_proto" in result.stdout
                and match is not None
            ):
                xyz = [
                    float(v) for v in match.group(1).split()
                ]
                rpy = [
                    float(v) for v in match.group(2).split()
                ]

                if (
                    len(xyz) == 3
                    and len(rpy) == 3
                    and all(
                        math.isfinite(v)
                        for v in xyz + rpy
                    )
                ):
                    return (xyz[0], xyz[1], rpy[2])

            last_error = (
                f"returncode={result.returncode} "
                f"stdout={result.stdout[-250:]!r} "
                f"stderr={result.stderr[-250:]!r}"
            )

        except (subprocess.TimeoutExpired, ValueError) as exc:
            last_error = repr(exc)

        print(
            f"WARN GAZEBO_POSE attempt={attempt}/4 "
            f"reason={last_error}",
            file=sys.stderr,
            flush=True,
        )

        if attempt < 4:
            time.sleep(0.75)

    raise RuntimeError(
        "GAZEBO_POSE_RETRIES_EXHAUSTED: "
        + str(last_error)
    )


class Monitor(Node):
    def __init__(self, active=False):
        super().__init__("pulse1_v6_sequencer")
        self.data = {}
        self.seen = {}

        for key, topic in (
            ("guard", "/pulse1/v6_motor_guard"),
            ("gf", "/pulse1/v6_geofence_watchdog"),
            ("sup", "/pulse1/v6_supervisor_status"),
        ):
            self.create_subscription(
                String, topic,
                lambda m, k=key: self.on_json(k, m), 10
            )

        self.create_subscription(
            Twist, CMD,
            lambda m: self.store(
                "cmd", (m.linear.x, m.angular.z)
            ), 10
        )
        self.create_subscription(
            WorldStatistics, "/world/empty/stats",
            lambda m: self.store("paused", bool(m.paused)),
            qos_profile_sensor_data
        )

        self.prepare_pub = (
            self.create_publisher(String, PREPARE, 10)
            if active else None
        )
        self.geo_pub = (
            self.create_publisher(String, GEO_REARM, 10)
            if active else None
        )

    def store(self, key, value):
        self.data[key] = value
        self.seen[key] = time.monotonic()

    def on_json(self, key, message):
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            value = None
        self.store(
            key, value if isinstance(value, dict) else None
        )

    def value(self, key):
        return self.data.get(key) or {}

    def fresh(self, key, limit=0.8):
        return (
            key in self.seen
            and time.monotonic() - self.seen[key] < limit
        )

    def poll(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        if self.value("guard").get("emergency_latched") is True:
            raise RuntimeError("GUARD_EMERGENCY")
        if self.value("sup").get("state") == "EMERGENCY_PAUSE":
            raise RuntimeError("SUPERVISOR_EMERGENCY")
        if self.value("gf").get("action") == "PAUSE":
            raise RuntimeError("GEOFENCE_ABORT")

    def wait(self, predicate, label, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if predicate():
                return
        raise RuntimeError("TIMEOUT_" + label)

    def base_safe(self):
        publishers = self.get_publishers_info_by_topic(CMD)
        cmd = self.data.get("cmd")
        return all((
            self.data.get("paused") is True,
            self.fresh("paused"),
            self.fresh("guard", 0.6),
            self.fresh("gf", 0.6),
            self.fresh("sup"),
            self.fresh("cmd", 0.6),
            isinstance(cmd, tuple),
            len(cmd) == 2 if isinstance(cmd, tuple) else False,
            all(math.isfinite(v) and abs(v) < 1e-8
                for v in cmd) if isinstance(cmd, tuple) else False,
            len(publishers) == 1,
            publishers[0].node_name == "pulse1_v6_motor_guard"
                if len(publishers) == 1 else False,
            len(self.get_publishers_info_by_topic(REQUEST)) == 0,
            self.value("sup").get("state") == "HEALTHY",
            self.value("guard").get("emergency_latched") is False,
        ))

    def stopped(self):
        g = self.value("guard")
        f = self.value("gf")
        sid = g.get("segment_id")
        return (
            self.base_safe()
            and isinstance(sid, str)
            and len(sid) >= 8
            and sid == f.get("segment_id")
            and g.get("nominal_stop_latched") is True
            and g.get("guard_state") == "BLOCKED"
            and f.get("action") == "COMPLETE"
            and f.get("reason") == "TARGET_REACHED"
            and f.get("pause_ack") is True
        )

    @staticmethod
    def publish(publisher, payload):
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False)
        publisher.publish(msg)

    def rearm(self, old_id, next_id):
        self.wait(
            lambda: self.stopped()
            and self.count_subscribers(PREPARE) >= 1
            and self.count_subscribers(GEO_REARM) >= 1,
            "REARM_PREFLIGHT"
        )

        if self.value("guard").get("segment_id") != old_id:
            raise RuntimeError("PREVIOUS_SEGMENT_CHANGED")

        payload = {
            "previous_segment_id": old_id,
            "next_segment_id": next_id,
            "operator_authorized": True,
        }

        self.publish(self.prepare_pub, payload)

        self.wait(
            lambda: self.base_safe()
            and self.value("guard").get("rearm_prepared_for") == next_id
            and self.value("guard").get(
                "rearm_previous_segment_id"
            ) == old_id
            and self.value("guard").get(
                "nominal_stop_latched"
            ) is True,
            "GUARD_PREPARE"
        )
        print("OK: GUARD PREPARED", flush=True)

        self.publish(self.geo_pub, payload)

        self.wait(
            lambda: self.base_safe()
            and self.value("gf").get("action") == "WAIT"
            and self.value("gf").get("reason") == "REARM_VERIFIED"
            and self.value("gf").get(
                "rearm_previous_segment_id"
            ) == old_id
            and self.value("gf").get(
                "rearm_next_segment_id"
            ) == next_id,
            "GEOFENCE_REARM"
        )
        print("OK: GEOFENCE REARMED", flush=True)

        self.wait(
            lambda: self.base_safe()
            and self.value("guard").get(
                "expected_next_segment_id"
            ) == next_id
            and self.value("guard").get(
                "nominal_stop_latched"
            ) is False
            and self.value("guard").get(
                "guard_state"
            ) == "BLOCKED"
            and self.value("gf").get("action") == "WAIT",
            "GUARD_COMMIT"
        )
        print("OK: GUARD COMMITTED", flush=True)


def run(mode):
    active = mode == "once"
    if active and os.environ.get(
        "PULSE1_V6_SEQUENCE_ARM"
    ) != "YES":
        raise RuntimeError("EXPLICIT_SEQUENCE_ARM_REQUIRED")

    rclpy.init(args=[])
    node = Monitor(active)
    old_id = None
    next_id = "v6-route-" + secrets.token_hex(10)

    try:
        node.wait(node.stopped, "VERIFIED_STOP")
        old_id = node.value("guard")["segment_id"]

        pose = gazebo_pose()
        node.wait(node.stopped, "STOP_AFTER_GAZEBO_POSE")

        if node.value("guard").get("segment_id") != old_id:
            raise RuntimeError("SEGMENT_CHANGED_DURING_PLAN")

        request = next_route_request(
            load_world(WORLD), pose, next_id, "LEFT"
        )

        print("VERIFIED STOP:", old_id, flush=True)
        print("GAZEBO POSE:", pose, flush=True)
        print(
            "NEXT:", request["kind"],
            "WAYPOINT:", request["waypoint_index"],
            "LIMIT:", round(request["limit"], 6),
            flush=True
        )

        if not active:
            print("PLAN ONLY: NO REARM / NO MOTION")
            return

        node.rearm(old_id, next_id)

        pose_after = gazebo_pose()
        if (
            math.hypot(
                pose_after[0] - pose[0],
                pose_after[1] - pose[1]
            ) > 0.015
            or abs(math.atan2(
                math.sin(pose_after[2] - pose[2]),
                math.cos(pose_after[2] - pose[2])
            )) > 0.04
        ):
            raise RuntimeError("POSE_CHANGED_AFTER_REARM")

    finally:
        node.destroy_node()
        rclpy.shutdown()

    env = dict(os.environ)
    env.update({
        "PULSE1_V6_EXECUTE": "YES",
        "PULSE1_V6_ROUTE_NEXT": "YES",
        "PULSE1_V6_SEGMENT_ID": next_id,
    })

    print("EXECUTING ONE SEGMENT:", next_id, flush=True)
    result = subprocess.run(
        [
            sys.executable, "-u",
            str(ROOT / "scripts/pulse1_v6_single_segment.py"),
            "--execute"
        ],
        cwd=ROOT, env=env, timeout=120
    )
    if result.returncode != 0:
        raise RuntimeError(
            "EXECUTOR_FAILED: do not retry or rearm"
        )

    rclpy.init(args=[])
    check = Monitor()
    try:
        check.wait(
            lambda: check.stopped()
            and check.value("guard").get("segment_id") == next_id,
            "FINAL_INDEPENDENT_STOP"
        )
    finally:
        check.destroy_node()
        rclpy.shutdown()

    print("SUCCESS: SEGMENT + INDEPENDENT STOP", next_id)


def main():
    parser = argparse.ArgumentParser()
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--plan", action="store_true")
    modes.add_argument("--once", action="store_true")
    args = parser.parse_args()

    try:
        run("once" if args.once else "plan")
    except Exception as exc:
        print("STOP SEQUENCER:", exc, file=sys.stderr)
        print(
            "NON RIPETERE / NON RIARMARE SENZA VERIFICA ROS",
            file=sys.stderr
        )
        raise SystemExit(1)


if __name__ == "__main__":
    main()
