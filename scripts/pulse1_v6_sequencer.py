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
from ros_gz_interfaces.srv import ControlWorld
from pulse1_v6_sensor_preflight import check_sensor_gate

from pulse1_waypoint_plan import load_world
from pulse1_v6_route_policy import next_route_request
from pulse1_v6_preflight_retry import (
    refresh_with_budget, check_replan_or_refresh, MAX_REFRESH_ATTEMPTS,
)

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
            ("observer", "/pulse1/avoid_observer"),
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
            self.on_stats, qos_profile_sensor_data
        )
        # Only active sequencers can request controlled, PAUSED sensor steps.
        self.control_client = (
            self.create_client(ControlWorld, '/world/empty/control')
            if active else None
        )

        self.prepare_pub = (
            self.create_publisher(String, PREPARE, 10)
            if active else None
        )
        self.geo_pub = (
            self.create_publisher(String, GEO_REARM, 10)
            if active else None
        )

    def on_stats(self, msg):
        self.store("paused", bool(msg.paused))
        self.store("world_stats", (int(msg.iterations), bool(msg.paused)))

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

    def sensor_gate(self, kind):
        age = (time.monotonic() - self.seen["observer"]
               if "observer" in self.seen else None)
        return check_sensor_gate(self.value("observer"), age, kind)

    def verify_quiescent_sensor_world(self):
        """Two fresh paused statistics with unchanged iterations before step."""
        current = self.data.get("world_stats")
        if not (self.stopped() and isinstance(current, tuple)
                and current[1] is True and self.fresh("world_stats")):
            raise RuntimeError("SENSOR_WORLD_NOT_QUIESCENT")
        baseline, _ = current
        seen = self.seen.get("world_stats")
        fresh_reports = 0
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.poll()
            current = self.data.get("world_stats")
            if (not self.stopped() or not isinstance(current, tuple)
                    or current[1] is not True or current[0] != baseline):
                raise RuntimeError("SENSOR_WORLD_CHANGED_BEFORE_STEP")
            stamp = self.seen.get("world_stats")
            if stamp != seen:
                fresh_reports += 1
                seen = stamp
                if fresh_reports >= 2:
                    return
        raise RuntimeError("SENSOR_WORLD_STABILITY_TIMEOUT")

    def refresh_sensors_before_rearm(self, kind):
        """Refresh sensors while an independently STOPPED segment is latched.

        Any timeout, unexpected ControlWorld step, lost STOP or clearance
        rejection halts BEFORE rearm. Never retry a pending service request.
        """
        if self.control_client is None:
            raise RuntimeError("SENSOR_PREFLIGHT_NOT_ACTIVE")
        self.wait(self.stopped, "SENSOR_STOP_REQUIRED")
        self.wait(lambda: self.control_client.service_is_ready(),
                  "SENSOR_CONTROL_DISCOVERY", timeout=4)

        # A paused world cannot generate fresh scans. Controlled multi_step
        # with the guard nominally latched produces SENSOR data without motion.
        for count in (250, 250, 500, 1000):
            self.wait(self.stopped, "SENSOR_STABLE_STOP", timeout=3)
            self.verify_quiescent_sensor_world()
            before = self.data["world_stats"]
            req = ControlWorld.Request()
            req.world_control.pause = True
            req.world_control.multi_step = count
            future = self.control_client.call_async(req)
            deadline = time.monotonic() + 25
            ack = False
            while time.monotonic() < deadline:
                self.poll()
                stats = self.data.get("world_stats")
                if future.done() and not ack:
                    result = future.result()
                    if result is None or result.success is not True:
                        raise RuntimeError("SENSOR_STEP_ACK_DENIED")
                    ack = True
                if stats is not None and stats[0] > before[0] + count + 50:
                    raise RuntimeError("SENSOR_STEP_OVERRUN")
                if ack and stats is not None and stats[1] is True \
                        and stats[0] >= before[0] + count:
                    # Do not repeat even a partially accepted control step.
                    self.wait(self.stopped, "SENSOR_STOP_AFTER_STEP", timeout=3)
                    break
            else:
                raise RuntimeError("SENSOR_STEP_PENDING_OR_TIMEOUT")
            # Observer reports asynchronously, independent of Gazebo ACK.
            for _ in range(40):
                self.poll()
                ready, reason = self.sensor_gate(kind)
                if ready:
                    print("SENSOR PREFLIGHT READY:", kind, reason, flush=True)
                    return
                if reason != "OBSERVER_NOT_HEALTHY":
                    print("SENSOR PREFLIGHT DENIED:", reason, flush=True)
                    raise RuntimeError("SENSOR_PREFLIGHT_DENIED:" + reason)
            print("SENSOR PREFLIGHT WAIT:", reason, flush=True)
        raise RuntimeError("SENSOR_PREFLIGHT_NO_FRESH_DATA")

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


def precision_wp0_required(request, gz_pose):
    """Narrow opt-in for existing fail-closed WP0 precision policy."""
    from pulse1_waypoint_plan import build_route
    corner_x = build_route(
        load_world(WORLD), "LEFT"
    )["waypoints"][0][0]
    return (
        isinstance(request, dict)
        and request.get("kind") == "ADVANCE"
        and request.get("waypoint_index") == 0
        and type(request.get("limit")) in (int, float)
        and math.isfinite(request["limit"])
        and 0.020 <= request["limit"] <= 0.035
        and isinstance(gz_pose, (tuple, list))
        and len(gz_pose) == 3
        and all(
            type(v) in (int, float) and math.isfinite(v)
            for v in gz_pose
        )
        and corner_x - 0.04 <= gz_pose[0] <= corner_x - 0.01
        and abs(gz_pose[1]) <= 0.01
        and abs(gz_pose[2]) <= 0.035
    )



def v6_corner_planning_pose(gz_pose, world, side):
    """Calibrated wheel-axle route reference at ANY heading.

    Uses the wheel geometry validated at WP0. Never changes raw Gazebo
    pose, real ROS odometry, Guard or Geofence state. Route corridor is
    checked by next_route_request and the two-frame handoff.
    """
    import xml.etree.ElementTree as ET
    from pulse1_v6_route_policy import build_route

    if not (
        isinstance(gz_pose, (tuple, list))
        and len(gz_pose) == 3
        and all(
            type(v) in (int, float) and math.isfinite(v)
            for v in gz_pose
        )
    ):
        raise RuntimeError("CORNER_FRAME_POSE_INVALID")

    x, y, yaw = gz_pose

    if side not in ("LEFT", "RIGHT"):
        raise RuntimeError("CORNER_FRAME_SIDE_INVALID")

    model_path = (
        ROOT / "simulation/models/pulse1/model_demo2.sdf"
    )
    model = ET.parse(model_path).getroot()

    wheels = []
    for name in ("left_wheel", "right_wheel"):
        element = model.find(
            f".//link[@name='{name}']/pose"
        )
        if element is None or not element.text:
            raise RuntimeError("CORNER_WHEEL_POSE_MISSING")
        coords = [float(v) for v in element.text.split()]
        if len(coords) != 6:
            raise RuntimeError("CORNER_WHEEL_POSE_INVALID")
        wheels.append(coords)

    left, right = wheels

    if not (
        abs(left[0] - right[0]) < 0.002
        and abs(left[1] + right[1]) < 0.002
        and abs((left[1] - right[1]) - 1.25) < 0.01
    ):
        raise RuntimeError("CORNER_AXLE_GEOMETRY_CHANGED")

    offset = (left[0] + right[0]) / 2.0

    if not 0.50 <= offset <= 0.60:
        raise RuntimeError("CORNER_AXLE_OFFSET_INVALID")

    virtual_pose = (
        x + offset * (math.cos(yaw) - 1.0),
        y + offset * math.sin(yaw),
        yaw,
    )

    # Validate the generalized rigid-transform reference against the route.
    # A paused rover may be rotating at a corner, so leave full heading and
    # corridor validation to next_route_request, never expand the corridor.
    if not all(math.isfinite(v) for v in virtual_pose):
        raise RuntimeError("CORNER_FRAME_NONFINITE")
    next_route_request(world, virtual_pose, "v6-frame-audit", side)
    return virtual_pose


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

        world_data = load_world(WORLD)
        planning_pose = v6_corner_planning_pose(
            pose, world_data, "LEFT"
        )
        planning_time = time.monotonic()
        request = next_route_request(
            world_data, planning_pose, next_id, "LEFT"
        )
        if planning_pose != pose:
            print(
                "CORNER FRAME COMPENSATED:",
                planning_pose,
                flush=True,
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

        # DO NOT commit Guard/Geofence until the specific maneuver has
        # recent, valid clearance. This step does not publish motor requests.
        initial_sensor_odom, refreshes_used = refresh_with_budget(
            node, request["kind"], MAX_REFRESH_ATTEMPTS, phase="INITIAL")
        node.wait(node.stopped, "STOP_AFTER_SENSOR_PREFLIGHT")
        refreshed_pose = gazebo_pose()
        node.wait(node.stopped, "STOP_AFTER_SENSOR_POSE")
        if (math.hypot(refreshed_pose[0] - pose[0],
                       refreshed_pose[1] - pose[1]) > 0.015
                or abs(math.atan2(math.sin(refreshed_pose[2] - pose[2]),
                                   math.cos(refreshed_pose[2] - pose[2]))) > 0.04):
            raise RuntimeError("POSE_CHANGED_DURING_SENSOR_PREFLIGHT")
        # Recalculate the plan from the final paused Gazebo snapshot BEFORE
        # committing; an honest corner transition requires an updated plan.
        pose = refreshed_pose
        planning_pose = v6_corner_planning_pose(pose, world_data, "LEFT")
        request = next_route_request(world_data, planning_pose, next_id, "LEFT")
        # Five refresh calls MAX across both preflight stages. A blocking
        # gz model call can age a healthy paused-world sensor snapshot.
        # Recover ONLY from stale data and check stationary odometry before
        # rearm; never retry a safety failure or an uncertain physics step.
        refreshes_used = check_replan_or_refresh(
            node, request["kind"], initial_sensor_odom, refreshes_used)
        planning_time = time.monotonic()
        print("SENSOR PREFLIGHT REPLAN:", request["kind"],
              "WP", request["waypoint_index"], flush=True)
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
    # Non ereditare flag precision da una shell precedente.
    for key in (
        "PULSE1_V6_PRECISION_WP0",
        "PULSE1_V6_PRECISION_GZ_POSE",
        "PULSE1_V6_PRECISION_GZ_TIME",
        "PULSE1_V6_ROUTE_GZ_POSE",
        "PULSE1_V6_ROUTE_GZ_TIME",
    ):
        env.pop(key, None)

    env.update({
        "PULSE1_V6_EXECUTE": "YES",
        "PULSE1_V6_ROUTE_NEXT": "YES",
        "PULSE1_V6_SEGMENT_ID": next_id,
        "PULSE1_V6_EXPECT_KIND": request["kind"],
        "PULSE1_V6_EXPECT_WP": str(request["waypoint_index"]),
        "PULSE1_V6_EXPECT_LIMIT": repr(request["limit"]),
        # Match the original plan chosen BEFORE rearm, with the same
        # frame snapshot. The post-rearm raw pose was checked <15 mm.
        "PULSE1_V6_ROUTE_GZ_POSE": ",".join(repr(v) for v in planning_pose),
        "PULSE1_V6_ROUTE_GZ_TIME": repr(planning_time),
    })

    if precision_wp0_required(request, pose_after):
        env["PULSE1_V6_PRECISION_WP0"] = "YES"
        env["PULSE1_V6_PRECISION_GZ_POSE"] = ",".join(
            repr(float(v)) for v in pose_after
        )
        env["PULSE1_V6_PRECISION_GZ_TIME"] = repr(
            time.monotonic()
        )
        print(
            "PRECISION WP0 ENABLED: "
            "Gazebo snapshot + odometry agreement required",
            flush=True,
        )

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
    modes.add_argument("--auto", action="store_true")
    args = parser.parse_args()

    try:
        if args.auto:
            if os.environ.get("PULSE1_V6_SEQUENCE_ARM") != "YES":
                raise RuntimeError("EXPLICIT_SEQUENCE_ARM_REQUIRED")
            for count in range(1, 251):
                try:
                    run("once")
                except ValueError as exc:
                    if str(exc) == "ROUTE_COMPLETE":
                        print("ROUTE COMPLETE: final nominal stop verified")
                        return
                    raise
                print(f"AUTO: SEGMENT {count}/250 CERTIFIED", flush=True)
            raise RuntimeError("AUTO_SEGMENT_LIMIT_EXCEEDED")
        else:
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
