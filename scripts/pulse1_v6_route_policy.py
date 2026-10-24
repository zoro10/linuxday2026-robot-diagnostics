"""S1 V6 route segment selection. Pure data, NO motor actuation."""

import math

from pulse1_waypoint_plan import (
    build_route, segment_hits_aabb
)
from pulse1_v6_safety_contract import (
    pose_valid, valid_request,
    MAX_FORWARD_M, MAX_LINEAR_M_S
)
from pulse1_v6_turn_policy import TURN_LIMIT, TURN_SPEED

MAX_ADVANCE = min(0.10, MAX_FORWARD_M)
ADVANCE_SPEED = min(0.10, MAX_LINEAR_M_S)
CORRIDOR_M = 0.025  # TEMPORANEO: SOLO GAZEBO
CORNER_REACHED_M = 0.020
HEADING_TOL_RAD = 0.075


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def next_route_request(world, pose, segment_id, side="LEFT"):
    """Return a proposal, NOT permission to move."""
    if not (
        pose_valid(pose)
        and isinstance(segment_id, str)
        and 8 <= len(segment_id) <= 64
        and side in ("LEFT", "RIGHT")
        and MAX_ADVANCE > 0
        and ADVANCE_SPEED > 0
        and TURN_LIMIT > 0
        and TURN_SPEED > 0
    ):
        raise ValueError("ROUTE_INPUT_INVALID")

    route = build_route(world, side)
    waypoints = route["waypoints"]

    if len(waypoints) != 5:
        raise ValueError("ROUTE_WAYPOINT_COUNT_CHANGED")

    points = [(0.0, 0.0)] + [
        (float(p[0]), float(p[1])) for p in waypoints
    ]

    x, y, yaw = pose
    candidates = []

    for i in range(len(points) - 1):
        start = points[i]
        end = points[i + 1]
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)

        if length <= 0.10:
            raise ValueError("ROUTE_LEG_TOO_SHORT")

        ux, uy = dx / length, dy / length
        rx, ry = x - start[0], y - start[1]
        along = rx * ux + ry * uy
        cross = abs(-rx * uy + ry * ux)
        remaining = length - along

        if (
            -CORRIDOR_M <= along <= length + CORRIDOR_M
            and cross <= (0.200 if side == "LEFT" and i == 2 and along >= 0.25 else CORRIDOR_M)
            and remaining > CORNER_REACHED_M
            and (i == 0 or along >= 0.0 or math.hypot(rx, ry) <= 0.05)
        ):
            candidates.append(
                (i, end, ux, uy, remaining, cross)
            )

    if not candidates:
        if math.hypot(
            x - points[-1][0], y - points[-1][1]
        ) <= CORNER_REACHED_M:
            raise ValueError("ROUTE_COMPLETE")
        raise ValueError("OUTSIDE_ROUTE_CORRIDOR")

    # Prevent premature turns where widened route corridors overlap.
    if len(candidates) > 1:
        next_leg = max(candidates, key=lambda c: c[0])
        prev_leg = next(
            (c for c in candidates if c[0] == next_leg[0] - 1),
            None,
        )
        if prev_leg is not None:
            corner = points[next_leg[0]]
            far = math.hypot(x - corner[0], y - corner[1]) > 0.05
            prev_yaw = math.atan2(prev_leg[3], prev_leg[2])
            next_yaw = math.atan2(next_leg[3], next_leg[2])
            if far and abs(wrap(yaw - prev_yaw)) < abs(wrap(yaw - next_yaw)):
                candidates.remove(next_leg)

    # At a corner, prefer the outgoing leg.
    i, target, ux, uy, remaining, cross = max(
        candidates, key=lambda c: c[0]
    )

    # Along must refer to the SELECTED route leg.
    # The loop variable 'along' belongs to the
    # last leg inspected and must not be reused.
    selected_start = points[i]
    selected_along = (
        (x - selected_start[0]) * ux
        + (y - selected_start[1]) * uy
    )

    desired_yaw = math.atan2(uy, ux)

    # WP2->WP3: compensate lateral drift using 1 m lookahead.
    # Planning only: actual motion still requires safety authorization.
    if side == "LEFT" and i == 2 and selected_along >= 0.25:
        lateral_error = (
            (y - selected_start[1]) * ux
            - (x - selected_start[0]) * uy
        )
        desired_yaw = wrap(
            desired_yaw - math.atan2(lateral_error, 1.0)
        )

    delta = wrap(desired_yaw - yaw)

    if abs(delta) > math.pi / 2 + 0.15:
        raise ValueError("HEADING_OUTSIDE_TURN_ENVELOPE")

    request = {
        "source": "v6_segment_executor",
        "mode": "AVOID",
        "motor_commands": False,
        "phase": "DRIVE",
        "segment_id": segment_id,
        "waypoint_index": i,
        "start_pose": [float(x), float(y), float(yaw)],
        "target": list(target),
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
    }

    bounds = route["inflated_obstacle_bounds"]

    # Correzione iniziale del solo tratto LEFT WP0->WP1.
    # Non modifica la tolleranza globale del planner.
    wp1_initial_alignment = (
        side == "LEFT"
        and i == 1
        and -0.015 <= selected_along <= 0.015
        and cross <= CORRIDOR_M
        and math.radians(1.0) < delta
        <= math.radians(3.0)
    )

    wp2_heading_correction = (
        side == "LEFT" and i == 2
        and selected_along >= 0.10
        and cross <= 0.10
        and math.radians(1.0) < abs(delta)
    )
    if abs(delta) > HEADING_TOL_RAD or wp1_initial_alignment or wp2_heading_correction:
        if segment_hits_aabb((x, y), (x, y), bounds):
            raise ValueError("TURN_INSIDE_KEEPOUT")

        direction = "LEFT" if delta > 0 else "RIGHT"
        request.update({
            "kind": "TURN",
            "direction": direction,
            "limit": min(abs(delta), TURN_LIMIT),
            "proposed_linear_m_s": 0.0,
            "proposed_angular_rad_s": (
                TURN_SPEED if delta > 0 else -TURN_SPEED
            ),
        })
    else:
        # Use shorter segments near a waypoint rather than
        # blindly driving the full 10 cm.
        limit = min(MAX_ADVANCE, remaining)
        destination = (
            x + limit * math.cos(yaw),
            y + limit * math.sin(yaw)
        )

        if segment_hits_aabb(
            (x, y), destination, bounds
        ):
            raise ValueError("ADVANCE_INTERSECTS_KEEPOUT")

        request.update({
            "kind": "ADVANCE",
            "direction": "",
            "limit": limit,
            "proposed_linear_m_s": ADVANCE_SPEED,
            "proposed_angular_rad_s": 0.0,
        })

    if not valid_request(request):
        raise ValueError("ROUTE_REQUEST_INVALID")

    return request
