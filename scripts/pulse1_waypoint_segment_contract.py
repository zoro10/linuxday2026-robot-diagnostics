#!/usr/bin/env python3
"""PULSE-1 S1: proposta FAIL-CLOSED di microsegmenti waypoint, SOLO SHADOW.

Non modifica Motor Guard o Geofence e NON autorizza un comando motore.
Un executor futuro deve avere watchdog/armamento, PAUSE/ACK, limiti
geometrici e conferma di STOP indipendenti.
"""
import math
from pulse1_navigation_math import wrap_angle
from pulse1_waypoint_plan import (
    HEADING_TOLERANCE_RAD, WAYPOINT_TOLERANCE_M,
    finite, front_verified, segment_hits_aabb, valid_observer, valid_pose,
)

MAX_FORWARD_SEGMENT_M = 0.20
MAX_TURN_SEGMENT_RAD = math.radians(15.0)
MIN_FRONT_CLEARANCE_M = 1.45   # prova geometrica del settore frontale, non certificazione anticollisione
MAX_AGE_S = 0.60
MAX_WORLD_AGE_S = 1.0
EPS_ZERO = 1e-8


def _zero(value):
    return finite(value) and abs(value) <= EPS_ZERO


def _turn_clearance(observer):
    """Replica prudenziale del criterio del TurnNavigator esistente.

    Sono indispensabili sei settori (3 front + 3 rear); senza un ritorno
    finito non inferiamo clearance garantita.
    """
    try:
        readings = observer['readings']
        sectors_by_sensor = [readings[s]['sectors'] for s in ('front', 'rear')]
        values = []
        for sectors in sectors_by_sensor:
            for name in ('front', 'left', 'right'):
                entry = sectors[name]
                if entry.get('clear') is True:
                    values.append(math.inf)
                else:
                    value = entry.get('distance_m')
                    if not finite(value) or value <= 0:
                        return None
                    values.append(value)
        nearest = min(values)
        return max(0.0, nearest-1.10) if math.isfinite(nearest) else None
    except (KeyError, TypeError, AttributeError):
        return None


def propose_segment(*, route, waypoint, waypoint_age_s, observer,
                    observer_age_s, guard, guard_age_s, cmd,
                    cmd_age_s, world_paused, world_age_s):
    """Ritorna un possibile microsegmento (nessun comando / nessuna autorizzazione).

    'candidate' è una proposta del livello superiore, NON un permesso operativo.
    """
    result = {
        'source': 'waypoint_segment_shadow', 'mode': 'AVOID',
        'state': 'HOLD', 'reason': 'NOT_READY', 'candidate': None,
        'motion_authorized': False, 'motor_commands': False,
        'proposed_linear_m_s': 0.0, 'proposed_angular_rad_s': 0.0,
    }

    def hold(reason):
        result['reason'] = reason
        return result

    if not isinstance(route, dict) or route.get('scenario') != 's1' \
            or route.get('mode') != 'AVOID' or route.get('motion_authorized') is not False \
            or route.get('side') not in ('LEFT', 'RIGHT'):
        return hold('INVALID_ROUTE')
    points = route.get('waypoints')
    box = route.get('inflated_obstacle_bounds')
    if not isinstance(points, list) or len(points) != 5 or not all(
        isinstance(p, list) and len(p) == 2 and all(finite(v) for v in p)
        for p in points
    ) or not isinstance(box, list) or len(box) != 4 or not all(finite(v) for v in box):
        return hold('INVALID_GEOMETRY')
    if world_paused is not True or not finite(world_age_s) or not 0 <= world_age_s <= MAX_WORLD_AGE_S:
        return hold('WORLD_NOT_PAUSED')
    if not finite(guard_age_s) or not 0 <= guard_age_s <= MAX_AGE_S \
            or not isinstance(guard, dict) or guard.get('source') != 'motor_guard_live' \
            or guard.get('mode') != 'AVOID' or guard.get('motor_commands') is not True \
            or guard.get('guard_state') not in ('BLOCKED', 'STALE_REQUEST') \
            or not _zero(guard.get('permitted_linear_m_s')) \
            or not _zero(guard.get('permitted_angular_rad_s')):
        return hold('GUARD_NOT_ZERO')
    if not finite(cmd_age_s) or not 0 <= cmd_age_s <= MAX_AGE_S \
            or not isinstance(cmd, (tuple, list)) or len(cmd) != 2 \
            or not all(_zero(v) for v in cmd):
        return hold('CMD_NOT_ZERO')
    if not valid_observer(observer, observer_age_s):
        return hold('OBSERVER_UNAVAILABLE')
    if not finite(waypoint_age_s) or not 0 <= waypoint_age_s <= MAX_AGE_S \
            or not isinstance(waypoint, dict):
        return hold('WAYPOINT_STALE')
    if waypoint.get('source') != 'waypoint_shadow' or waypoint.get('mode') != 'AVOID' \
            or waypoint.get('side') != route['side'] or waypoint.get('motor_commands') is not False \
            or waypoint.get('motion_authorized') is not False \
            or not _zero(waypoint.get('proposed_linear_m_s')) \
            or not _zero(waypoint.get('proposed_angular_rad_s')):
        return hold('INVALID_WAYPOINT_SOURCE')
    if waypoint.get('state') == 'ARRIVED':
        if waypoint.get('waypoint_index') == len(points):
            result.update(state='ARRIVED', reason='WAYPOINTS_COMPLETE')
            return result
        return hold('INVALID_ARRIVAL')
    if waypoint.get('state') != 'PLAN':
        return hold('WAYPOINT_HOLD')
    i = waypoint.get('waypoint_index')
    if type(i) is not int or not 0 <= i < len(points) or waypoint.get('waypoint_count') != len(points):
        return hold('INVALID_INDEX')
    target = points[i]
    if waypoint.get('target') != target:
        return hold('TARGET_CHANGED')
    pose = waypoint.get('pose')
    if not valid_pose(pose):
        return hold('INVALID_POSE')
    readings = observer.get('readings')
    odom = readings.get('odom') if isinstance(readings, dict) else None
    if not isinstance(odom, dict) or not valid_pose((odom.get('x'), odom.get('y'), odom.get('yaw_rad'))):
        return hold('OBSERVER_POSE_INVALID')
    observer_pose = (odom['x'], odom['y'], odom['yaw_rad'])
    if math.hypot(pose[0] - observer_pose[0], pose[1] - observer_pose[1]) > 0.05 \
            or abs(wrap_angle(pose[2] - observer_pose[2])) > 0.05:
        return hold('POSES_DISAGREE')
    distance = math.hypot(target[0]-pose[0], target[1]-pose[1])
    heading = wrap_angle(math.atan2(target[1]-pose[1], target[0]-pose[0])-pose[2])
    if distance <= WAYPOINT_TOLERANCE_M:
        return hold('WAYPOINT_TRANSITION_PENDING')
    if not finite(waypoint.get('distance_m')) or abs(distance-waypoint['distance_m']) > 0.055 \
            or not finite(waypoint.get('heading_error_rad')) \
            or abs(wrap_angle(heading-waypoint['heading_error_rad'])) > 0.05:
        return hold('WAYPOINT_DATA_INCONSISTENT')
    if segment_hits_aabb((pose[0], pose[1]), (pose[0], pose[1]), box):
        return hold('INSIDE_KEEPOUT')

    candidate = {'waypoint_index': i, 'start_pose': [round(v, 6) for v in pose],
                 'target': target.copy(), 'requires_independent_stop_ack': True}
    if abs(heading) > HEADING_TOLERANCE_RAD:
        if waypoint.get('suggested_action') != ('TURN_LEFT' if heading > 0 else 'TURN_RIGHT'):
            return hold('TURN_DIRECTION_MISMATCH')
        clearance = _turn_clearance(observer)
        if clearance is None:
            return hold('TURN_CLEARANCE_UNAVAILABLE')
        if clearance < 2.10:
            return hold('TURN_CLEARANCE_TOO_SMALL')
        candidate.update(kind='TURN', direction='LEFT' if heading > 0 else 'RIGHT',
                         max_angle_rad=round(min(abs(heading), MAX_TURN_SEGMENT_RAD), 6))
    else:
        if waypoint.get('suggested_action') != 'ADVANCE_CANDIDATE':
            return hold('ADVANCE_MISMATCH')
        known, front = front_verified(observer)
        if not known or front < MIN_FRONT_CLEARANCE_M:
            return hold('FRONT_NOT_CLEAR')
        length = min(MAX_FORWARD_SEGMENT_M, distance-WAYPOINT_TOLERANCE_M)
        if length <= 0:
            return hold('ZERO_LENGTH')
        end = (pose[0] + length*math.cos(pose[2]), pose[1] + length*math.sin(pose[2]))
        if segment_hits_aabb((pose[0], pose[1]), end, box):
            return hold('SEGMENT_INTERSECTS_KEEPOUT')
        candidate.update(kind='ADVANCE', max_distance_m=round(length, 6),
                         front_distance_m=front if math.isfinite(front) else 'CLEAR')
    result.update(state='CANDIDATE', reason='OFFLINE_PROPOSAL_ONLY', candidate=candidate)
    return result
