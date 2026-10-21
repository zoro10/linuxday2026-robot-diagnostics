"""Conservative *first* S1 V6 segment contract. Pure Python; no actuation."""
import math
import secrets
from pulse1_waypoint_plan import build_route, valid_pose, segment_hits_aabb
from pulse1_v6_safety_contract import valid_request

LENGTH_M = 0.10
SPEED_M_S = 0.10


def first_request(world, pose, segment_id=None):
    """Produce a well-formed drive request ONLY as data; never permission."""
    route = build_route(world, 'LEFT')
    if segment_id is None:
        segment_id = 'v6-first-' + secrets.token_hex(10)
    if not valid_pose(pose) or math.hypot(pose[0], pose[1]) > 0.03 \
            or abs(pose[2]) > 0.06:
        raise ValueError('S1_REQUIRES_INITIAL_POSE')
    candidate = {
        'source': 'v6_segment_executor', 'mode': 'AVOID', 'motor_commands': False,
        'phase': 'DRIVE', 'segment_id': segment_id, 'waypoint_index': 0,
        'kind': 'ADVANCE', 'direction': '',
        'start_pose': [float(v) for v in pose],
        'target': route['waypoints'][0], 'limit': LENGTH_M,
        'proposed_linear_m_s': SPEED_M_S, 'proposed_angular_rad_s': 0.0,
        'odom_ok': True, 'lidar_ok': True, 'rear_lidar_ok': True, 'imu_ok': True,
    }
    if not valid_request(candidate):
        raise ValueError('INVALID_V6_REQUEST')
    return candidate



def resumed_first_leg_request(world, pose, segment_id):
    """Prossimi 10 cm sul primo rettilineo S1; nessuna autorizzazione."""
    route = build_route(world, 'LEFT')

    if not valid_pose(pose):
        raise ValueError('RESUME_POSE_INVALID')
    if not isinstance(segment_id, str) or not 8 <= len(segment_id) <= 64:
        raise ValueError('RESUME_SEGMENT_ID_INVALID')

    x, y, yaw = pose
    first_waypoint = route['waypoints'][0]

    if not (
        0.045 <= x < first_waypoint[0] - LENGTH_M - 0.04
        and abs(y) <= 0.03
        and abs(yaw) <= 0.06
    ):
        raise ValueError('RESUME_OUTSIDE_VERIFIED_FIRST_LEG')

    end = (
        x + LENGTH_M * math.cos(yaw),
        y + LENGTH_M * math.sin(yaw)
    )
    if segment_hits_aabb(
        (x, y), end, route['inflated_obstacle_bounds']
    ):
        raise ValueError('RESUME_INTERSECTS_KEEPOUT')

    request = {
        'source': 'v6_segment_executor',
        'mode': 'AVOID',
        'motor_commands': False,
        'phase': 'DRIVE',
        'segment_id': segment_id,
        'waypoint_index': 0,
        'kind': 'ADVANCE',
        'direction': '',
        'start_pose': [float(v) for v in pose],
        'target': first_waypoint,
        'limit': LENGTH_M,
        'proposed_linear_m_s': SPEED_M_S,
        'proposed_angular_rad_s': 0.0,
        'odom_ok': True,
        'lidar_ok': True,
        'rear_lidar_ok': True,
        'imu_ok': True,
    }
    if not valid_request(request):
        raise ValueError('RESUME_REQUEST_DENIED')
    return request


def stop_request(drive):
    if not valid_request(drive):
        raise ValueError('INVALID_DRIVE_REQUEST')
    return {**drive, 'phase': 'HOLD', 'reason': 'SEGMENT_TARGET_REACHED',
            'proposed_linear_m_s': 0.0, 'proposed_angular_rad_s': 0.0}


def start_ok(*, world_paused, world_age, cmd, cmd_age, guard, guard_age,
             watchdog, watchdog_age, v5_nodes, publisher_count,
             request_publishers, service_ready, supervisor, supervisor_age):
    """Read-only V6 handover predicate. Never infers permissions from V5."""
    def recent(v, limit):
        return type(v) in (int, float) and math.isfinite(v) and 0 <= v <= limit
    if not isinstance(guard, dict) or not isinstance(watchdog, dict) or not isinstance(supervisor, dict):
        return False
    return all((
        world_paused is True, recent(world_age, .8),
        isinstance(cmd, (list, tuple)), len(cmd or ()) == 2,
        all(type(x) in (int, float) and math.isfinite(x) and abs(x) <= 1e-8
            for x in (cmd or ())), recent(cmd_age, .6),
        isinstance(guard, dict), recent(guard_age, .6),
        guard.get('source') == 'motor_guard_live', guard.get('v6_armed') is True,
        guard.get('guard_state') == 'BLOCKED', guard.get('motor_commands') is True,
        guard.get('cmd_vel_exclusive') is True,
        guard.get('emergency_latched') is False,
        guard.get('nominal_stop_latched') is False,
        guard.get('permitted_linear_m_s') == 0,
        guard.get('permitted_angular_rad_s') == 0,
        isinstance(watchdog, dict), recent(watchdog_age, .6),
        watchdog.get('source') == 'v6_geofence_watchdog',
        watchdog.get('armed') is True, watchdog.get('action') == 'WAIT',
        watchdog.get('ready_for_motion') is False,
        v5_nodes is False, publisher_count == 1,
        request_publishers == 1, service_ready is True,
        isinstance(supervisor, dict), recent(supervisor_age, .8),
        supervisor.get('source') == 'v6_supervisor',
        supervisor.get('armed') is True,
        supervisor.get('state') == 'HEALTHY',
    ))


def progress_ok(pose, origin, length=LENGTH_M):
    if not valid_pose(pose) or not valid_pose(origin):
        return False
    dx, dy = pose[0]-origin[0], pose[1]-origin[1]
    along = dx*math.cos(origin[2]) + dy*math.sin(origin[2])
    cross = -dx*math.sin(origin[2]) + dy*math.cos(origin[2])
    angle = math.atan2(math.sin(pose[2]-origin[2]), math.cos(pose[2]-origin[2]))
    return (-.01 <= along <= length+.015 and abs(cross) <= .035
            and abs(angle) <= .09)


def reached(pose, origin, length=LENGTH_M):
    if not progress_ok(pose, origin, length):
        return False
    along = (pose[0]-origin[0])*math.cos(origin[2]) + (pose[1]-origin[1])*math.sin(origin[2])
    return along >= length-.012


def step_count(pose, origin, dt, length=LENGTH_M):
    """Bound physics steps assuming <= 0.1m/s, with 1.5x conservatism."""
    if not progress_ok(pose, origin, length) or not (
            type(dt) in (int, float) and .0001 <= dt <= .0011):
        raise ValueError('INVALID_STEP_STATE')
    along = (pose[0]-origin[0])*math.cos(origin[2]) + (pose[1]-origin[1])*math.sin(origin[2])
    remaining = length+.015-along
    n = min(50, int(remaining / (SPEED_M_S*dt*1.5)))
    if n < 1:
        raise ValueError('NO_SAFE_PHYSICS_BUDGET')
    return n


def should_emit_nominal_hold(watchdog, drive, pose):
    """Only watchdog-confirmed target permits transition DRIVE -> HOLD."""
    return (isinstance(watchdog, dict) and valid_request(drive)
            and watchdog.get('source') == 'v6_geofence_watchdog'
            and watchdog.get('action') == 'STOPPING'
            and watchdog.get('reason') == 'TARGET_REACHED'
            and watchdog.get('segment_id') == drive['segment_id']
            and watchdog.get('kind') == drive['kind']
            and watchdog.get('waypoint_index') == drive['waypoint_index']
            and reached(pose, drive['start_pose'], drive['limit']))
