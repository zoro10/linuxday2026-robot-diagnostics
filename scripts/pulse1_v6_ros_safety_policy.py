"""Pure adapter policy for ROS V6 watchdog/guard.

Does not create ROS publishers and does not authorize motion on its own.
"""
from pulse1_v6_safety_contract import (
    observer_healthy, pose_valid, recent, valid_request,
    MAX_ODOM_AGE_S, MAX_OBSERVER_AGE_S,
)


def watchdog_lease(*, result, arm_enabled, request, request_age_s,
                   observer, observer_age_s, pose, pose_age_s,
                   guard, guard_age_s, cmd_age_s, world_age_s,
                   pause_service_ready):
    """Only a live ROS owner with independent inputs can turn a candidate into a lease."""
    # Disarmed startup and missing/malformed requests are normal.  Never
    # evaluate request.get() until the request is validated; all((...))
    # constructs its tuple eagerly, so an earlier False is not sufficient.
    if (not isinstance(result, dict) or not isinstance(guard, dict)
            or not valid_request(request)):
        return False
    return all((
        arm_enabled is True,
        result.get('action') == 'MONITOR',
        result.get('reason') == 'WITHIN_SEGMENT_ENVELOPE',
        result.get('candidate_for_motion') is True,
        recent(request_age_s, .35),
        result.get('segment_id') == request.get('segment_id'),
        result.get('kind') == request.get('kind'),
        result.get('waypoint_index') == request.get('waypoint_index'),
        observer_healthy(observer, observer_age_s),
        pose_valid(pose), recent(pose_age_s, MAX_ODOM_AGE_S),
        guard.get('source') == 'motor_guard_live',
        guard.get('mode') == 'AVOID',
        guard.get('motor_commands') is True,
        guard.get('guard_state') in ('BLOCKED', 'AUTHORIZED'),
        guard.get('v6_armed') is True,
        guard.get('cmd_vel_exclusive') is True,
        guard.get('emergency_latched') is False,
        recent(guard_age_s, .35),
        recent(cmd_age_s, .35),
        recent(world_age_s, .6),
        pause_service_ready is True,
    ))


def motor_guard_status(*, armed, velocity, request, pause_ready, exclusive):
    """Guard status for the independent V6 watchdog; never claim active if unsafe."""
    linear, angular = velocity
    active = (armed is True and exclusive is True and pause_ready is True
              and (linear != 0 or angular != 0))
    return {
        'source': 'motor_guard_live', 'mode': 'AVOID',
        'guard_state': 'AUTHORIZED' if active else 'BLOCKED',
        'v6_armed': armed is True,
        'motor_commands': armed is True,
        'cmd_vel_exclusive': exclusive is True,
        'segment_id': request.get('segment_id') if active and isinstance(request, dict) else None,
        'permitted_linear_m_s': linear if active else 0.0,
        'permitted_angular_rad_s': angular if active else 0.0,
    }


def nominal_v6_stop(*, watchdog, watchdog_age_s, request, active_segment_id):
    """Accept a nominal STOP only from fresh, matching V6 watchdog evidence.

    Never grants motion. If this returns True, the caller MUST latch zero and
    require an independent restart/STOP ACK for the next segment.
    """
    if (not isinstance(watchdog, dict) or
            not isinstance(request, dict) or
            type(active_segment_id) is not str or
            not 8 <= len(active_segment_id) <= 64 or
            not recent(watchdog_age_s, .25)):
        return False
    if not (valid_request(request, moving=True) or
            valid_request(request, moving=False)):
        return False
    return (
        request.get('segment_id') == active_segment_id
        and watchdog.get('source') == 'v6_geofence_watchdog'
        and watchdog.get('mode') == 'AVOID'
        and watchdog.get('armed') is True
        and watchdog.get('motor_commands') is False
        and watchdog.get('action') == 'STOPPING'
        and watchdog.get('reason') == 'TARGET_REACHED'
        and watchdog.get('segment_id') == active_segment_id
        and watchdog.get('kind') == request.get('kind')
        and watchdog.get('waypoint_index') == request.get('waypoint_index')
    )
