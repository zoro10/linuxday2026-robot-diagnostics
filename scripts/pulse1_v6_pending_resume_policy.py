"""One incident-specific V6 pending-rearm recovery; no ROS motor commands.

This policy is intentionally NOT a generic reset, and cannot certify a nominal
STOP. The snapshot is tied to the observed S1 WP2 incident and physical pose.
"""
import math
import re
import subprocess
import time

PENDING_ID = 'v6-route-5bd8fe220b5c73d92224'
LAST_CERTIFIED_ID = 'v6-route-96ee2814762ab5320d1b'
# Physically observed Gazebo model origin, after the failed pre-motion warmup.
PENDING_GAZEBO = (2.9208203403625337, 2.8213209207480694, 0.031596)
LAST_CERTIFIED_GAZEBO = (2.920260, 2.821320, 0.031596)
MAX_SNAPSHOT_AGE_S = 6 * 3600
REQUIRED_CHECKS = (
    'base_safe', 'guard_blocked', 'guard_pending', 'geofence_pending',
    'geofence_wait', 'no_nominal_stop', 'no_emergency',
    'supervisor_healthy', 'world_paused', 'cmd_zero',
)


def _pose(p):
    return (isinstance(p, (tuple, list)) and len(p) == 3
            and all(type(x) in (float, int) and math.isfinite(x) for x in p))


def _aligned(a, b, distance=.015, yaw=.02):
    return (_pose(a) and _pose(b)
            and math.hypot(a[0]-b[0], a[1]-b[1]) <= distance
            and abs(math.atan2(math.sin(a[2]-b[2]),
                               math.cos(a[2]-b[2]))) <= yaw)


def _zero(cmd):
    return (isinstance(cmd, (tuple, list)) and len(cmd) == 2
            and all(type(x) in (float, int) and math.isfinite(x)
                    and abs(x) <= 1e-8 for x in cmd))


def verify_old_pending_audit(audit, *, now=None):
    """Check historical evidence; this is NOT a motion/no-motion certificate."""
    if not isinstance(audit, dict):
        raise ValueError('PENDING_AUDIT_TYPE')
    now = time.time() if now is None else now
    at = audit.get('captured_at_epoch')
    if not (type(at) in (float, int) and math.isfinite(at)
            and type(now) in (float, int) and math.isfinite(now)
            and 0 <= now - at <= MAX_SNAPSHOT_AGE_S):
        raise ValueError('PENDING_AUDIT_STALE')
    checks = audit.get('checks')
    guard = audit.get('guard')
    gf = audit.get('geofence')
    sup = audit.get('supervisor')
    if not (audit.get('record_type') == 'V6_REARMED_PREMOTION'
            and audit.get('incident') == 'OUTSIDE_ROUTE_CORRIDOR_BEFORE_REQUEST'
            and audit.get('pending_segment_id') == PENDING_ID
            and audit.get('last_certified_segment_id') == LAST_CERTIFIED_ID
            and audit.get('nominal_stop') is False
            and audit.get('world_paused') is True
            and type(audit.get('world_iterations')) is int
            and audit['world_iterations'] >= 1
            and _zero(audit.get('cmd'))
            and isinstance(checks, dict)
            and all(checks.get(k) is True for k in REQUIRED_CHECKS)
            and isinstance(guard, dict) and isinstance(gf, dict)
            and isinstance(sup, dict)):
        raise ValueError('PENDING_AUDIT_INVALID')
    if not (
        guard.get('source') == 'motor_guard_live'
        and guard.get('v6_armed') is True
        and guard.get('cmd_vel_exclusive') is True
        and guard.get('motor_commands') is True
        and guard.get('guard_state') == 'BLOCKED'
        and guard.get('expected_next_segment_id') == PENDING_ID
        and guard.get('segment_id') in (None, '')
        and guard.get('rearm_prepared_for') in (None, '')
        and guard.get('nominal_stop_latched') is False
        and guard.get('emergency_latched') is False
        and guard.get('permitted_linear_m_s') == 0.0
        and guard.get('permitted_angular_rad_s') == 0.0
        and gf.get('source') == 'v6_geofence_watchdog'
        and gf.get('armed') is True
        and gf.get('action') == 'WAIT'
        and gf.get('reason') == 'REARM_VERIFIED'
        and gf.get('segment_id') in (None, '')
        and gf.get('rearm_previous_segment_id') == LAST_CERTIFIED_ID
        and gf.get('rearm_next_segment_id') == PENDING_ID
        and gf.get('ready_for_motion') is False
        and gf.get('candidate_for_motion') is False
        and gf.get('pause_error') is None
        and sup.get('source') == 'v6_supervisor'
        and sup.get('armed') is True
        and sup.get('state') == 'HEALTHY'
        and sup.get('motor_commands') is False
    ):
        raise ValueError('PENDING_AUDIT_NOT_QUIESCENT')
    if not _aligned(PENDING_GAZEBO, LAST_CERTIFIED_GAZEBO, .015, .02):
        raise ValueError('PENDING_REFERENCE_NOT_ALIGNED')
    return True


def verify_pending_bootstrap(audit, guard, gf, sup, paused, cmd, gz_pose,
                             *, now=None):
    """Accept only a clean NEW V6 session and same physical Gazebo pose."""
    verify_old_pending_audit(audit, now=now)
    if not (_aligned(gz_pose, PENDING_GAZEBO, .015, .02)
            and _aligned(gz_pose, LAST_CERTIFIED_GAZEBO, .025, .025)):
        raise ValueError('PENDING_GAZEBO_POSE_CHANGED')
    if not (paused is True and _zero(cmd)
            and isinstance(guard, dict) and isinstance(gf, dict)
            and isinstance(sup, dict)
            and guard.get('source') == 'motor_guard_live'
            and guard.get('guard_state') == 'BLOCKED'
            and guard.get('v6_armed') is True
            and guard.get('motor_commands') is True
            and guard.get('cmd_vel_exclusive') is True
            and guard.get('emergency_latched') is False
            and guard.get('nominal_stop_latched') is False
            and guard.get('segment_id') in (None, '')
            and guard.get('expected_next_segment_id') in (None, '')
            and guard.get('rearm_prepared_for') in (None, '')
            and guard.get('permitted_linear_m_s') == 0.0
            and guard.get('permitted_angular_rad_s') == 0.0
            and gf.get('source') == 'v6_geofence_watchdog'
            and gf.get('armed') is True
            and gf.get('action') == 'WAIT'
            and gf.get('reason') == 'NOT_STARTED'
            and gf.get('segment_id') in (None, '')
            and gf.get('rearm_next_segment_id') in (None, '')
            and gf.get('ready_for_motion') is False
            and gf.get('pause_error') is None
            and sup.get('source') == 'v6_supervisor'
            and sup.get('armed') is True
            and sup.get('state') == 'HEALTHY'
            and sup.get('motor_commands') is False):
        raise ValueError('PENDING_BOOTSTRAP_NOT_CLEAN')
    return True


def parse_gz_model_pose(raw):
    """Read model origin from Gazebo's text protobuf topic (not ROS odom)."""
    if not isinstance(raw, str):
        raise ValueError('PENDING_POSE_UNREADABLE')
    records = re.findall(r'(?ms)^pose \{\s*\n\s*name: "pulse1_proto"(.*?)^\}', raw)
    if not records:
        raise ValueError('PENDING_POSE_NOT_FOUND')
    poses = []
    for part in records:
        pm = re.search(r'position\s*\{([^{}]*)\}', part)
        qm = re.search(r'orientation\s*\{([^{}]*)\}', part)
        if not pm or not qm:
            raise ValueError('PENDING_POSE_MISSING_COMPONENTS')
        def scalar(txt, field, default=None):
            m = re.search(r'(?m)^\s*' + re.escape(field)
                          + r':\s*([-+\d.eE]+)\s*$', txt)
            if m is None:
                if default is not None:
                    return default
                raise ValueError('PENDING_POSE_FIELD_MISSING_' + field)
            return float(m.group(1))
        x = scalar(pm.group(1), 'x')
        y = scalar(pm.group(1), 'y')
        qx = scalar(qm.group(1), 'x', 0.0)
        qy = scalar(qm.group(1), 'y', 0.0)
        qz = scalar(qm.group(1), 'z', 0.0)
        qw = scalar(qm.group(1), 'w', 0.0)
        qnorm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if not (0.99 <= qnorm <= 1.01):
            raise ValueError('PENDING_POSE_QUATERNION')
        yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        p = (x, y, yaw)
        if not _pose(p):
            raise ValueError('PENDING_POSE_NONFINITE')
        poses.append(p)
    # Reject Gazebo outputs with internal motion disagreement.
    if not all(_aligned(p, poses[0], .002, .005) for p in poses):
        raise ValueError('PENDING_POSE_TOPIC_UNSTABLE')
    return poses[-1]


def gazebo_pose_from_topic():
    """Read-only. A paused world may not terminate its topic publisher."""
    try:
        proc = subprocess.run(['gz', 'topic', '-e', '-t', '/world/empty/pose/info'],
                              capture_output=True, text=True, timeout=5,
                              check=False)
        raw = proc.stdout
    except subprocess.TimeoutExpired as exc:
        raw = exc.stdout or b''
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', 'replace')
    if len(raw) > 2_000_000:
        raise ValueError('PENDING_POSE_OUTPUT_TOO_LARGE')
    return parse_gz_model_pose(raw)
