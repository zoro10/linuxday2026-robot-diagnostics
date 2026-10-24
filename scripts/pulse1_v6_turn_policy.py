"""V6 TURN policy: pure functions, no ROS or motor actuation."""
import math

from pulse1_waypoint_plan import build_route
from pulse1_v6_safety_contract import (
    MAX_TURN_RAD,
    MAX_ANGULAR_RAD_S,
    pose_valid,
    valid_request,
)

TURN_LIMIT = min(math.radians(12), round(MAX_TURN_RAD, 6))
TURN_SPEED = min(0.12, MAX_ANGULAR_RAD_S)
MAX_POSITION_DRIFT = 0.035
MAX_ANGLE_OVERSHOOT = math.radians(1.5)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def signed_turn(pose, origin, direction):
    delta = wrap(pose[2] - origin[2])
    return delta if direction == "LEFT" else -delta


def turn_progress_ok(pose, origin, direction, limit):
    if not (
        pose_valid(pose)
        and pose_valid(origin)
        and direction in ("LEFT", "RIGHT")
        and type(limit) in (int, float)
        and math.isfinite(limit)
        and 0 < limit <= TURN_LIMIT
    ):
        return False

    progress = signed_turn(pose, origin, direction)
    return (
        math.hypot(
            pose[0] - origin[0],
            pose[1] - origin[1]
        ) <= MAX_POSITION_DRIFT
        and -MAX_ANGLE_OVERSHOOT <= progress
        <= limit + MAX_ANGLE_OVERSHOOT
    )


def turn_reached(pose, origin, direction, limit):
    # A micro-TURN cannot be completed below the
    # minimum angular progress required by the FSM.
    # Preserve the existing geometric completion
    # checks and reject only premature completion.
    if turn_progress_ok(pose, origin, direction, limit):
        from pulse1_v6_segment_fsm import MIN_PROGRESS_RAD

        if signed_turn(
            pose, origin, direction
        ) < MIN_PROGRESS_RAD:
            return False

    return (
        turn_progress_ok(pose, origin, direction, limit)
        and signed_turn(pose, origin, direction)
        >= limit - math.radians(2)
    )


def turn_step_count(pose, origin, direction, limit, dt):
    if not (
        turn_progress_ok(pose, origin, direction, limit)
        and type(dt) in (int, float)
        and math.isfinite(dt)
        and 0.0001 <= dt <= 0.0011
        and TURN_SPEED > 0
        and not turn_reached(pose, origin, direction, limit)
    ):
        raise ValueError("TURN_STEP_DENIED")

    remaining = limit - signed_turn(pose, origin, direction)
    budget = int(
        (remaining + 0.004) / (TURN_SPEED * dt * 1.5)
    )
    return max(1, min(40, budget))


def turn_request(world, pose, segment_id, direction):
    """Only at the first planned corner; never grants permission."""
    if (
        not pose_valid(pose)
        or direction not in ("LEFT", "RIGHT")
        or not isinstance(segment_id, str)
        or not 8 <= len(segment_id) <= 64
        or TURN_LIMIT <= 0
        or TURN_SPEED <= 0
    ):
        raise ValueError("TURN_INPUT_DENIED")

    route = build_route(world, direction)
    points = route["waypoints"]

    if len(points) < 2:
        raise ValueError("TURN_ROUTE_INCOMPLETE")

    corner, following = points[0], points[1]

    if math.hypot(
        pose[0] - corner[0],
        pose[1] - corner[1]
    ) > 0.04:
        raise ValueError("TURN_NOT_AT_PLANNED_CORNER")

    heading = math.atan2(
        following[1] - corner[1],
        following[0] - corner[0]
    )
    remaining = wrap(heading - pose[2])
    signed_remaining = (
        remaining if direction == "LEFT" else -remaining
    )

    if not math.radians(4) <= signed_remaining <= math.pi / 2 + 0.10:
        raise ValueError("TURN_HEADING_NOT_ALLOWED")

    limit = min(TURN_LIMIT, signed_remaining)
    speed = TURN_SPEED if direction == "LEFT" else -TURN_SPEED

    request = {
        "source": "v6_segment_executor",
        "mode": "AVOID",
        "motor_commands": False,
        "phase": "DRIVE",
        "segment_id": segment_id,
        "waypoint_index": 0,
        "kind": "TURN",
        "direction": direction,
        "start_pose": list(pose),
        "target": following,
        "limit": limit,
        "proposed_linear_m_s": 0.0,
        "proposed_angular_rad_s": speed,
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
    }

    if not valid_request(request):
        raise ValueError("TURN_REQUEST_INVALID")

    return request
