#!/usr/bin/env python3
"""Preview geometrica S1 V6. NON pubblica ROS e NON muove il rover."""

import json
import math
from pathlib import Path

from pulse1_waypoint_plan import (
    build_route, load_world, segment_hits_aabb
)
from pulse1_navigation_math import wrap_angle

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "runtime/pulse1_world.json"
OUTPUT = ROOT / "runtime/pulse1_mission_preview.json"

MAX_FORWARD = 0.20
MAX_TURN = math.radians(15)
EPS = 1e-9


def generate(world, initial_pose, side="LEFT"):
    route = build_route(world, side)
    box = route["inflated_obstacle_bounds"]

    x, y, yaw = initial_pose
    steps = []

    for index, target in enumerate(route["waypoints"]):
        tx, ty = target
        desired = math.atan2(ty - y, tx - x)

        # Rotazioni sul posto, ciascuna di massimo 15 gradi.
        for _ in range(100):
            error = wrap_angle(desired - yaw)
            if abs(error) <= EPS:
                break

            angle = math.copysign(
                min(abs(error), MAX_TURN), error
            )

            steps.append({
                "kind": "TURN",
                "waypoint_index": index,
                "direction": "LEFT" if angle > 0 else "RIGHT",
                "limit": round(abs(angle), 6),
                "start_pose": [x, y, yaw],
                "expected_pose": [x, y, wrap_angle(yaw + angle)],
                "motion_authorized": False
            })
            yaw = wrap_angle(yaw + angle)
        else:
            raise ValueError("Rotazione non convergente")

        # Avanzamenti di massimo 20 cm.
        for _ in range(1000):
            distance = math.hypot(tx - x, ty - y)
            if distance <= EPS:
                break

            length = min(distance, MAX_FORWARD)
            nx = x + length * math.cos(yaw)
            ny = y + length * math.sin(yaw)

            if segment_hits_aabb((x, y), (nx, ny), box):
                raise ValueError(
                    f"Segmento {index} interseca il keepout"
                )

            steps.append({
                "kind": "ADVANCE",
                "waypoint_index": index,
                "limit": round(length, 6),
                "start_pose": [x, y, yaw],
                "expected_pose": [nx, ny, yaw],
                "motion_authorized": False
            })
            x, y = nx, ny
        else:
            raise ValueError("Avanzamento non convergente")

    return {
        "mission": "S1_AVOID",
        "state": "PREVIEW_ONLY",
        "motion_authorized": False,
        "checkpoint_source": "HISTORICAL_APPROXIMATION",
        "start_pose": list(initial_pose),
        "route": route,
        "total_segments": len(steps),
        "forward_segments": sum(
            s["kind"] == "ADVANCE" for s in steps
        ),
        "turn_segments": sum(
            s["kind"] == "TURN" for s in steps
        ),
        "segments": steps
    }


if __name__ == "__main__":
    world = load_world(WORLD)
    plan = generate(world, (0.0899, 0.0, 0.0))

    assert plan["motion_authorized"] is False
    assert plan["total_segments"] > 0

    for step in plan["segments"]:
        assert step["motion_authorized"] is False
        if step["kind"] == "ADVANCE":
            assert 0 < step["limit"] <= 0.20
        else:
            assert 0 < step["limit"] <= 0.261800

    temporary = OUTPUT.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(plan, indent=2, allow_nan=False)
    )
    temporary.replace(OUTPUT)

    print("MISSIONE:", plan["mission"])
    print("STATO:", plan["state"])
    print("MICROSEGMENTI:", plan["total_segments"])
    print("AVANZAMENTI:", plan["forward_segments"])
    print("ROTAZIONI:", plan["turn_segments"])
    print("WAYPOINT:", len(plan["route"]["waypoints"]))
    print("LUNGHEZZA ROTTA:", plan["route"]["path_length_m"], "m")
    print("FILE:", OUTPUT)
    print("MOTORI: NON AUTORIZZATI")
