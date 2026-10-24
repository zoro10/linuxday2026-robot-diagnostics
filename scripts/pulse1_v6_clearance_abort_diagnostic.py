"""Read-only, fail-safe snapshot of the *exact* observer used at a V6 clearance abort.

No ROS dependencies. This module never changes an authorization or a safety
predicate. Diagnostics are deliberately frozen at the first abort transition.
"""
import math

from pulse1_v6_safety_contract import turn_clearance_detail


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        return None
    return float(value)


def _sectors(observer):
    readings = observer.get('readings') if isinstance(observer, dict) else None
    readings = readings if isinstance(readings, dict) else {}
    output = {}
    for sensor in ('front', 'rear'):
        reading = readings.get(sensor)
        raw = reading.get('sectors') if isinstance(reading, dict) else None
        raw = raw if isinstance(raw, dict) else {}
        output[sensor] = {}
        for sector in ('front', 'left', 'right'):
            item = raw.get(sector)
            if not isinstance(item, dict):
                output[sensor][sector] = {'clear': None, 'distance_m': None}
            else:
                clear = item.get('clear')
                output[sensor][sector] = {
                    'clear': clear if type(clear) is bool else None,
                    'distance_m': _number(item.get('distance_m')),
                }
    return output


def clearance_abort_snapshot(*, result, observer, observer_age_s,
                              pose, now_s):
    """Snapshot original inputs to the Geofence MONITOR clearance failure.

    Caller must invoke synchronously on the MONITOR -> PAUSE transition.
    Returns None outside OBSTACLE_CLEARANCE_LOST. Never calls a motor API.
    """
    if (not isinstance(result, dict)
            or result.get('action') != 'PAUSE'
            or result.get('reason') != 'OBSTACLE_CLEARANCE_LOST'):
        return None

    observer = observer if isinstance(observer, dict) else {}
    kind = result.get('kind')
    if kind == 'TURN':
        _ok, detail = turn_clearance_detail(observer)
    elif kind == 'ADVANCE':
        reading = observer.get('readings')
        front = reading.get('front') if isinstance(reading, dict) else None
        front = front if isinstance(front, dict) else {}
        detail = 'ADVANCE_FRONT_CLEARANCE_DENIED'
        if front:
            detail += '_DIST_' + str(_number(front.get('distance_m')))
    else:
        detail = 'UNKNOWN_MANEUVER'

    ages = observer.get('ages_s')
    ages = ages if isinstance(ages, dict) else {}
    values = [_number(v) for v in pose] if isinstance(pose, (tuple, list)) and len(pose) == 3 else []
    return {
        'record_type': 'V6_CLEARANCE_ABORT_SNAPSHOT',
        'geofence_reason': 'OBSTACLE_CLEARANCE_LOST',
        'clearance_detail': detail,
        'segment_id': result.get('segment_id'),
        'kind': kind,
        'waypoint_index': result.get('waypoint_index'),
        'pose_odom': values if len(values) == 3 and all(v is not None for v in values) else None,
        'observer_state': observer.get('state') if type(observer.get('state')) is str else None,
        'observer_ros_age_s': _number(observer_age_s),
        'observer_sensor_ages_s': {key: _number(ages.get(key)) for key in ('odom', 'front', 'rear', 'imu')},
        'sensor_flags': {key: observer.get(key) is True for key in ('odom_ok', 'lidar_ok', 'rear_lidar_ok', 'imu_ok')},
        'sectors': _sectors(observer),
        'geofence_monotonic_s': _number(now_s),
    }
