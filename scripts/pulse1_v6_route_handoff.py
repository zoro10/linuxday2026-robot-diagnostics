"""Pure two-frame route handoff. No ROS, actuator, or safety authorization.

A corner bridge may only propose a small, geometrically verified ADVANCE.
The motor guard, geofence, supervisor and segment FSM remain authoritative.
"""
import math

from pulse1_waypoint_plan import build_route, segment_hits_aabb
from pulse1_v6_route_policy import (
    CORNER_REACHED_M, CORRIDOR_M, HEADING_TOL_RAD,
    MAX_ADVANCE, next_route_request, wrap,
)
from pulse1_v6_safety_contract import pose_valid, valid_request

MAX_FRAME_GAP_M = 0.045
MAX_YAW_GAP_RAD = 0.05
MIN_BRIDGE_M = 0.020
BRIDGE_MARGIN_M = 0.003
MAX_OVERSHOOT_M = 0.018  # inside 25 mm corridor, with 7 mm margin


def _shift(pose, distance):
    x, y, yaw = pose
    return (x + distance * math.cos(yaw),
            y + distance * math.sin(yaw), yaw)


def reconcile_route_plan(world, route_pose, odom_pose, segment_id, side="LEFT"):
    """Return (request, physical_plan, mode), failing closed on disagreement.

    The planned request starts at *real ROS odometry*; the Gazebo route pose is
    ONLY a geometric reference, never forged into the sensor data.
    """
    if not (pose_valid(route_pose) and pose_valid(odom_pose)):
        raise ValueError("HANDOFF_POSE_INVALID")
    if (math.hypot(route_pose[0] - odom_pose[0],
                   route_pose[1] - odom_pose[1]) > MAX_FRAME_GAP_M
            or abs(wrap(route_pose[2] - odom_pose[2])) > MAX_YAW_GAP_RAD):
        raise ValueError("HANDOFF_FRAME_DISAGREEMENT")

    physical = next_route_request(world, route_pose, segment_id, side)
    odometric = next_route_request(world, odom_pose, segment_id, side)
    same = (physical['kind'] == odometric['kind']
            and physical['waypoint_index'] == odometric['waypoint_index']
            and physical['direction'] == odometric['direction'])
    if same:
        result = dict(physical)
        result['start_pose'] = list(odom_pose)
        result['limit'] = min(physical['limit'], odometric['limit'])
        if not valid_request(result):
            raise ValueError("HANDOFF_REQUEST_INVALID")
        return result, physical, 'AGREED'

    # Only a disagreement across one adjacent corner may be reconciled.
    plans = (physical, odometric)
    advancing = [p for p in plans if p['kind'] == 'ADVANCE']
    turning = [p for p in plans if p['kind'] == 'TURN']
    if len(advancing) != 1 or len(turning) != 1:
        raise ValueError("HANDOFF_NOT_ADJACENT_CORNER")
    forward, corner = advancing[0], turning[0]
    if corner['waypoint_index'] != forward['waypoint_index'] + 1:
        raise ValueError("HANDOFF_NOT_ADJACENT_CORNER")

    # Micro-advance only, in the previous leg's heading. No forced turn.
    if abs(wrap(route_pose[2] - odom_pose[2])) > HEADING_TOL_RAD:
        raise ValueError("HANDOFF_HEADING_DISAGREEMENT")
    needed = forward['limit'] - CORNER_REACHED_M + BRIDGE_MARGIN_M
    limit = max(MIN_BRIDGE_M, needed)
    if not (0 < limit <= MAX_ADVANCE and math.isfinite(limit)):
        raise ValueError("HANDOFF_BRIDGE_TOO_LONG")

    # The bridge must leave BOTH independent pose estimates ready to TURN
    # toward the same next waypoint. Validate complete end positions, too.
    after = []
    for pose in (route_pose, odom_pose):
        destination = _shift(pose, limit)
        r = next_route_request(world, destination, segment_id, side)
        if (r['kind'] != 'TURN'
                or r['waypoint_index'] != corner['waypoint_index']
                or r['direction'] != corner['direction']):
            raise ValueError("HANDOFF_BRIDGE_NOT_CONVERGENT")
        after.append(destination)

    route = build_route(world, side)
    bounds = route['inflated_obstacle_bounds']
    if any(segment_hits_aabb(p[:2], dest[:2], bounds)
           for p, dest in zip((route_pose, odom_pose), after)):
        raise ValueError("HANDOFF_BRIDGE_KEEPOUT")

    # Both odometry and route estimates may overshoot the old corner by only
    # a small amount; this avoids treating broad corridor overlap as consent.
    points = [(0.0, 0.0)] + [tuple(map(float, p)) for p in route['waypoints']]
    target = points[forward['waypoint_index'] + 1]
    start = points[forward['waypoint_index']]
    length = math.dist(start, target)
    ux, uy = (target[0] - start[0]) / length, (target[1] - start[1]) / length
    for dest in after:
        remaining = (target[0] - dest[0]) * ux + (target[1] - dest[1]) * uy
        cross = abs(-(dest[0] - start[0]) * uy + (dest[1] - start[1]) * ux)
        if remaining < -MAX_OVERSHOOT_M or cross > CORRIDOR_M:
            raise ValueError("HANDOFF_BRIDGE_OUT_OF_BOUNDS")

    result = dict(forward)
    result['start_pose'] = list(odom_pose)
    result['limit'] = limit
    if not valid_request(result):
        raise ValueError("HANDOFF_BRIDGE_INVALID")
    return result, physical, 'CORNER_BRIDGE'
