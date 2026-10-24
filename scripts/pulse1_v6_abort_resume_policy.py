#!/usr/bin/env python3
"""Fail-closed audit of one operator-authorized V6 fresh-session bootstrap.

Never changes a ROS node, motor output, nominal latch, or Geofence state.
An ABORT is permanent within the old session. This only verifies that the
robot has not moved while the operator deliberately restarted V6 nodes.
"""
import math
import time

ABORT_ID = "v6-route-3485bc37278b517a05cc"
ABORT_REASON = "START_SENSORS_OR_CLEARANCE"
# Last displayed physical Gazebo pose at WP1, before the rejected TURN.
WP1_GAZEBO = (2.64830, 2.25795, 1.56237)
# A SEPARATE, later live incident: partial TURN toward WP2. Never claim
# that this segment ended nominally. The model origin moves on a spot turn,
# but the wheel-axle center remains fixed within measured precision.
TURN_ABORT_ID = "v6-route-02f8670abf24130e7433"
TURN_ABORT_REASON = "OBSTACLE_CLEARANCE_LOST"
TURN_ABORT_GAZEBO = (2.479450, 2.286320, 1.252350)
# Separate mid-ADVANCE abort after eight certified sequencer segments at WP2.
WP2_ODOM_ABORT_ID = 'v6-route-9d161fe9973b5a5918f4'
WP2_ODOM_ABORT_REASON = 'ODOMETRY_STALE'
WP2_ODOM_ABORT_GAZEBO = (2.294610, 2.801490, 0.031558)
WP2_LAST_NOMINAL_GAZEBO = (2.27788, 2.80096, 0.031553)


def _pose(p):
    return (isinstance(p, (list, tuple)) and len(p) == 3
            and all(type(v) in (int, float) and math.isfinite(v) for v in p))


def _aligned(a, b, distance=0.015, yaw=0.02):
    return (_pose(a) and _pose(b)
            and math.hypot(a[0]-b[0], a[1]-b[1]) <= distance
            and abs(math.atan2(math.sin(a[2]-b[2]),
                                   math.cos(a[2]-b[2]))) <= yaw)


def _quiescent(guard, gf, supervisor, paused, cmd):
    return (isinstance(guard, dict) and isinstance(gf, dict)
            and isinstance(supervisor, dict) and paused is True
            and supervisor.get('state') == 'HEALTHY'
            and guard.get('guard_state') == 'BLOCKED'
            and guard.get('emergency_latched') is False
            and guard.get('nominal_stop_latched') is False
            and guard.get('v6_armed') is True
            and guard.get('cmd_vel_exclusive') is True
            and gf.get('armed') is True
            and gf.get('ready_for_motion') is False
            and isinstance(cmd, (list, tuple)) and len(cmd) == 2
            and all(type(v) in (int, float) and math.isfinite(v)
                    and abs(v) <= 1e-8 for v in cmd))


def capture_abort(guard, gf, supervisor, paused, cmd, gz_pose, *, now=None):
    """Store the ABORT as historical evidence; NEVER certify a nominal STOP."""
    if not _quiescent(guard, gf, supervisor, paused, cmd):
        raise ValueError('ABORT_AUDIT_NOT_QUIESCENT')
    if (guard.get('expected_next_segment_id') != ABORT_ID
            or guard.get('segment_id') not in (None, '')
            or gf.get('action') != 'PAUSE'
            or gf.get('reason') != ABORT_REASON):
        raise ValueError('ABORT_AUDIT_WRONG_STATE')
    if not _aligned(gz_pose, WP1_GAZEBO, .025, .025):
        raise ValueError('ABORT_AUDIT_GAZEBO_POSE_CHANGED')
    return {'record_type':'V6_ABORT_NOT_NOMINAL',
            'aborted_segment_id':ABORT_ID, 'reason':ABORT_REASON,
            'world_paused':True, 'cmd_zero':True,
            'motion_authorized':False,
            'gz_pose':[float(v) for v in gz_pose],
            'captured_at_epoch':float(time.time() if now is None else now)}



def capture_turn_abort(guard, gf, supervisor, paused, cmd, gz_pose, *, now=None):
    """Audit a *partially executed* segment that stopped in emergency.

    This is read-only: unlike a nominal stop, ABORT is never reclassified.
    This intentionally cannot run after replacing/restarting controller nodes.
    """
    if not (isinstance(guard,dict) and isinstance(gf,dict)
            and isinstance(supervisor,dict) and paused is True
            and isinstance(cmd,(tuple,list)) and len(cmd)==2
            and all(type(v) in (int,float) and math.isfinite(v)
                    and abs(v) <= 1e-8 for v in cmd)
            and guard.get('guard_state') == 'ABORT'
            and guard.get('emergency_latched') is True
            and guard.get('nominal_stop_latched') is False
            and guard.get('v6_armed') is True
            and guard.get('cmd_vel_exclusive') is True
            and guard.get('expected_next_segment_id') == TURN_ABORT_ID
            and gf.get('armed') is True
            and gf.get('ready_for_motion') is False
            and gf.get('action') == 'PAUSE'
            and gf.get('reason') == TURN_ABORT_REASON
            and gf.get('segment_id') == TURN_ABORT_ID
            and supervisor.get('state') == 'EMERGENCY_PAUSE'
            and supervisor.get('pause_ack') is True
            and supervisor.get('motor_commands') is False):
        raise ValueError('TURN_ABORT_NOT_CONFIRMED')
    if not _aligned(gz_pose, TURN_ABORT_GAZEBO, .015, .02):
        raise ValueError('TURN_ABORT_POSE_CHANGED')
    return {
        'record_type':'V6_ABORT_NOT_NOMINAL',
        'incident':'MID_TURN_CLEARANCE_ABORT',
        'aborted_segment_id':TURN_ABORT_ID,
        'reason':TURN_ABORT_REASON,
        'world_paused':True, 'cmd_zero':True,
        'motion_authorized':True,  # was authorized and moved before ABORT
        'partial_motion_observed':True,
        'nominal_stop':False,
        'gz_pose':[float(v) for v in gz_pose],
        'captured_at_epoch':float(time.time() if now is None else now),
    }

def capture_wp2_odom_abort(guard, gf, supervisor, paused, cmd, gz_pose, *, now=None):
    """Certify the *emergency evidence*, not the aborted segment's completion.

    Side-effect free; must run while the original ABORT latch still exists.
    """
    if not (isinstance(guard, dict) and isinstance(gf, dict)
            and isinstance(supervisor, dict) and paused is True
            and isinstance(cmd, (tuple, list)) and len(cmd) == 2
            and all(type(v) in (int, float) and math.isfinite(v)
                    and abs(v) <= 1e-8 for v in cmd)
            and guard.get('guard_state') == 'ABORT'
            and guard.get('emergency_latched') is True
            and guard.get('nominal_stop_latched') is False
            and guard.get('v6_armed') is True
            and guard.get('cmd_vel_exclusive') is True
            and guard.get('segment_id') in (None, '')
            and guard.get('expected_next_segment_id') == WP2_ODOM_ABORT_ID
            and guard.get('permitted_linear_m_s') == 0.0
            and guard.get('permitted_angular_rad_s') == 0.0
            and gf.get('armed') is True
            and gf.get('ready_for_motion') is False
            and gf.get('action') == 'PAUSE'
            and gf.get('reason') == WP2_ODOM_ABORT_REASON
            and gf.get('segment_id') == WP2_ODOM_ABORT_ID
            and gf.get('pause_ack') is True
            and gf.get('motor_commands') is False
            and supervisor.get('state') == 'EMERGENCY_PAUSE'
            and supervisor.get('pause_ack') is True
            and supervisor.get('motor_commands') is False):
        raise ValueError('WP2_ODOM_ABORT_NOT_CONFIRMED')
    if not _aligned(gz_pose, WP2_ODOM_ABORT_GAZEBO, .015, .02):
        raise ValueError('WP2_ODOM_ABORT_POSE_CHANGED')
    distance = math.hypot(gz_pose[0] - WP2_LAST_NOMINAL_GAZEBO[0],
                          gz_pose[1] - WP2_LAST_NOMINAL_GAZEBO[1])
    if not (0.008 <= distance <= 0.075):
        raise ValueError('WP2_ODOM_ABORT_PARTIAL_MOTION_NOT_PROVEN')
    return {
        'record_type': 'V6_ABORT_NOT_NOMINAL',
        'incident': 'MID_ADVANCE_ODOMETRY_STALE_ABORT',
        'aborted_segment_id': WP2_ODOM_ABORT_ID,
        'reason': WP2_ODOM_ABORT_REASON,
        'world_paused': True, 'cmd_zero': True,
        'motion_authorized': True,
        'partial_motion_observed': True,
        'nominal_stop': False,
        'gz_pose': [float(v) for v in gz_pose],
        'captured_at_epoch': float(time.time() if now is None else now),
    }


def verify_stationary_odom(before, after, distance=0.005, yaw=0.010):
    """Fail closed if the PAUSED sensor refresh displaced the rover.

    The motor Guard must remain BLOCKED; this independent odometry guard
    additionally detects a physical drift from the extra physics steps.
    """
    if not _aligned(before, after, distance, yaw):
        raise ValueError('BOOTSTRAP_SENSOR_REFRESH_ROVER_MOVED')
    return True


def verify_bootstrap(audit, guard, gf, supervisor, paused, cmd, gz_pose, *, now=None):
    """Only accept a clean NEW safety session and unchanged physical pose."""
    if not isinstance(audit, dict) or not _quiescent(guard, gf, supervisor, paused, cmd):
        raise ValueError('COLD_ROUTE_NOT_QUIESCENT')
    if audit.get('aborted_segment_id') == ABORT_ID:
        incident_valid = (audit.get('reason') == ABORT_REASON
                          and audit.get('motion_authorized') is False)
        reference_pose = WP1_GAZEBO
    elif audit.get('aborted_segment_id') == TURN_ABORT_ID:
        incident_valid = (audit.get('reason') == TURN_ABORT_REASON
                          and audit.get('incident') == 'MID_TURN_CLEARANCE_ABORT'
                          and audit.get('motion_authorized') is True
                          and audit.get('partial_motion_observed') is True
                          and audit.get('nominal_stop') is False)
        reference_pose = TURN_ABORT_GAZEBO
    elif audit.get('aborted_segment_id') == WP2_ODOM_ABORT_ID:
        incident_valid = (audit.get('reason') == WP2_ODOM_ABORT_REASON
                          and audit.get('incident') == 'MID_ADVANCE_ODOMETRY_STALE_ABORT'
                          and audit.get('motion_authorized') is True
                          and audit.get('partial_motion_observed') is True
                          and audit.get('nominal_stop') is False)
        reference_pose = WP2_ODOM_ABORT_GAZEBO
    else:
        incident_valid = False
        reference_pose = None
    if not (audit.get('record_type') == 'V6_ABORT_NOT_NOMINAL'
            and incident_valid
            and audit.get('world_paused') is True
            and audit.get('cmd_zero') is True):
        raise ValueError('COLD_ROUTE_INVALID_AUDIT')
    now = time.time() if now is None else now
    saved_at=audit.get('captured_at_epoch')
    if (type(saved_at) not in (int,float) or not math.isfinite(saved_at)
            or not 0 <= now-saved_at <= 21600):
        raise ValueError('COLD_ROUTE_AUDIT_STALE')
    if not (_aligned(gz_pose,audit.get('gz_pose'), .015, .02)
            and _aligned(gz_pose,reference_pose,.025,.025)):
        raise ValueError('COLD_ROUTE_GAZEBO_POSE_CHANGED')
    if not (guard.get('expected_next_segment_id') in (None,'')
            and guard.get('segment_id') in (None,'')
            and gf.get('action') == 'WAIT'
            and gf.get('reason') == 'NOT_STARTED'
            and gf.get('segment_id') in (None,'')
            and gf.get('rearm_next_segment_id') in (None,'')):
        raise ValueError('COLD_ROUTE_NOT_CLEAN_NEW_SESSION')
    return True
