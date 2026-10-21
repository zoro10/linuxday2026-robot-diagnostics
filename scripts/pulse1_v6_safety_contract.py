#!/usr/bin/env python3
"""PULSE-1 V6: independent pure contracts for one bounded segment.

NO ROS, Gazebo, file-system I/O, or motor publishers. This is an OFFLINE
safety policy draft, not an armable controller. It never grants motion alone.
V5 remains completely untouched.
"""
import math
from dataclasses import dataclass

MAX_FORWARD_M = 0.20
MAX_TURN_RAD = math.radians(15)
MAX_LINEAR_M_S = 0.10
MAX_ANGULAR_RAD_S = 0.30
MAX_REQUEST_AGE_S = 0.35
MAX_WATCHDOG_AGE_S = 0.25
MAX_ODOM_AGE_S = 0.60
MAX_OBSERVER_AGE_S = 0.60
MAX_SEGMENT_WALL_S = 30.0
MAX_STOP_WALL_S = 3.0
MAX_CROSS_TRACK_M = 0.035
MAX_OVERSHOOT_M = 0.015
MAX_OVERSHOOT_RAD = math.radians(2)
SENSOR_FLAGS = ('odom_ok', 'lidar_ok', 'rear_lidar_ok', 'imu_ok')


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def recent(age, bound):
    return finite(age) and 0 <= age <= bound


def pose_valid(p):
    return isinstance(p, (list, tuple)) and len(p) == 3 and all(finite(x) for x in p)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def zero_cmd(cmd):
    return isinstance(cmd, (list, tuple)) and len(cmd) == 2 and all(
        finite(v) and abs(v) <= 1e-8 for v in cmd)


def valid_request(d, *, moving=True):
    """Strict V6 request schema; malformed input must NEVER raise."""
    if not isinstance(d, dict):
        return False
    segment_id = d.get('segment_id')
    waypoint_index = d.get('waypoint_index')
    target = d.get('target')
    kind = d.get('kind')
    if not (
        d.get('source') == 'v6_segment_executor'
        and d.get('mode') == 'AVOID'
        and d.get('motor_commands') is False
        and d.get('phase') == ('DRIVE' if moving else 'HOLD')
        and type(segment_id) is str and 8 <= len(segment_id) <= 64
        and type(waypoint_index) is int and 0 <= waypoint_index < 5
        and kind in ('ADVANCE', 'TURN')
        and pose_valid(d.get('start_pose'))
        and isinstance(target, (list, tuple)) and len(target) == 2
        and all(finite(v) for v in target)
    ):
        return False
    linear, angular = d.get('proposed_linear_m_s'), d.get('proposed_angular_rad_s')
    limit = d.get('limit')
    if not all(finite(v) for v in (linear, angular, limit)):
        return False
    if not moving:
        return linear == 0 and angular == 0 and d.get('reason') == 'SEGMENT_TARGET_REACHED'
    if not all(d.get(flag) is True for flag in SENSOR_FLAGS):
        return False
    if kind == 'ADVANCE':
        return (0 < limit <= MAX_FORWARD_M and 0 < linear <= MAX_LINEAR_M_S
                and angular == 0 and d.get('direction') in ('', None))
    return (0 < limit <= round(MAX_TURN_RAD, 6) and linear == 0
            and 0 < abs(angular) <= MAX_ANGULAR_RAD_S
            and d.get('direction') == ('LEFT' if angular > 0 else 'RIGHT'))


def observer_healthy(observer, age_s):
    if not isinstance(observer, dict):
        return False
    return (
        recent(age_s, MAX_OBSERVER_AGE_S)
        and observer.get('source') == 'avoid_observer'
        and observer.get('mode') == 'AVOID'
        and observer.get('state') == 'SENSOR_READY'
        and observer.get('motor_commands') is False
        and all(observer.get(flag) is True for flag in SENSOR_FLAGS)
    )


def front_clear(observer):
    """Directly read the real S1 Observer front reading; unknown fails closed."""
    if not isinstance(observer, dict):
        return False
    readings = observer.get('readings')
    front = readings.get('front') if isinstance(readings, dict) else None
    if not isinstance(front, dict):
        return False
    if front.get('clear') is True:
        return front.get('distance_m') is None
    distance = front.get('distance_m')
    return finite(distance) and distance >= 1.45


def turn_clear(observer):
    """Require known readings from six sectors and ample turning clearance."""
    readings = observer.get('readings') if isinstance(observer, dict) else None
    if not isinstance(readings, dict):
        return False
    values = []
    for sensor in ('front', 'rear'):
        reading = readings.get(sensor)
        sectors = reading.get('sectors') if isinstance(reading, dict) else None
        if not isinstance(sectors, dict):
            return False
        for direction in ('front', 'left', 'right'):
            entry = sectors.get(direction)
            if not isinstance(entry, dict):
                return False
            if entry.get('clear') is True and entry.get('distance_m') is None:
                values.append(math.inf)
            elif finite(entry.get('distance_m')) and entry['distance_m'] > 0:
                values.append(entry['distance_m'])
            else:
                return False
    finite_readings = [v for v in values if math.isfinite(v)]
    # Do not infer guaranteed clearance if every sector is 'clear'.
    return bool(finite_readings) and min(finite_readings) >= 3.20


def geofence_permission(status, request, age_s):
    """Independent lease: guard must not accept permission for another segment."""
    if not valid_request(request) or not isinstance(status, dict):
        return False
    return (
        recent(age_s, MAX_WATCHDOG_AGE_S)
        and status.get('source') == 'v6_geofence_watchdog'
        and status.get('mode') == 'AVOID'
        and status.get('armed') is True
        and status.get('action') == 'MONITOR'
        and status.get('reason') == 'WITHIN_SEGMENT_ENVELOPE'
        and status.get('ready_for_motion') is True
        and status.get('motor_commands') is False
        and status.get('segment_id') == request['segment_id']
        and status.get('kind') == request['kind']
        and status.get('waypoint_index') == request['waypoint_index']
    )


def v6_guard_velocity(request, *, request_age_s, watchdog, watchdog_age_s,
                      observer, observer_age_s, odom_age_s):
    """Fail-closed output. No independent watchdog => ZERO/ZERO."""
    if not valid_request(request):
        return (0.0, 0.0)
    if not (
        recent(request_age_s, MAX_REQUEST_AGE_S)
        and geofence_permission(watchdog, request, watchdog_age_s)
        and recent(odom_age_s, MAX_ODOM_AGE_S)
        and observer_healthy(observer, observer_age_s)
        and (front_clear(observer) if request['kind'] == 'ADVANCE'
             else turn_clear(observer))
    ):
        return (0.0, 0.0)
    return (float(request['proposed_linear_m_s']),
            float(request['proposed_angular_rad_s']))


def stopped_independently(*, world_paused, world_age_s, guard,
                           guard_age_s, cmd, cmd_age_s,
                           strict_v6=False, nominal_segment_id=None):
    if not isinstance(guard, dict):
        return False
    return (
        world_paused is True
        and recent(world_age_s, .8)
        and guard.get('source') == 'motor_guard_live'
        and guard.get('mode') == 'AVOID'
        and guard.get('guard_state') in ('BLOCKED', 'STALE_REQUEST')
        and guard.get('motor_commands') is True
        and guard.get('permitted_linear_m_s') == 0
        and guard.get('permitted_angular_rad_s') == 0
        and recent(guard_age_s, .6)
        and zero_cmd(cmd)
        and recent(cmd_age_s, .6)
        and (not strict_v6 or (
            guard.get('v6_armed') is True
            and guard.get('cmd_vel_exclusive') is True
            and guard.get('emergency_latched') is False
        ))
        and (nominal_segment_id is None or (
            strict_v6 and guard.get('nominal_stop_latched') is True
            and guard.get('segment_id') == nominal_segment_id
        ))
    )


@dataclass
class V6GeofenceSegment:
    """Independent per-segment envelope monitor, with latched fail-close.

    Inputs are *observations*, never commands. ABORT is permanent; no replay.
    Intended for future ROS watchdog. Does not itself pause Gazebo: the ROS
    owner must send PAUSE and verify acknowledgement upon any alarm.
    """
    phase: str = 'WAIT'
    request_id: str = ''
    kind: str = ''
    waypoint_index: int = -1
    direction: str = ''
    target: tuple = None
    origin: tuple = None
    limit: float = 0.0
    started_s: float = None
    stop_at_s: float = None
    reason: str = 'NOT_STARTED'
    strict_v6_guard: bool = False  # ROS armed owner MUST enable this
    expected_next_segment_id: str = ''


    def rearm_after_confirmed_stop(
            self, *, previous_segment_id, next_segment_id,
            operator_authorized, previous_executor_exited,
            pause_ack, guard, guard_age_s, cmd, cmd_age_s,
            world_paused, world_age_s):
        """Riarmo condizionato; non autorizza il movimento.

        Il Motor Guard resta indipendentemente bloccato e richiede
        una propria transizione di riarmo verificata.
        """
        if self.strict_v6_guard is not True:
            return False
        if self.phase != 'COMPLETE' or self.reason != 'TARGET_REACHED':
            return False
        if (operator_authorized is not True
                or previous_executor_exited is not True
                or pause_ack is not True):
            return False
        if (type(previous_segment_id) is not str
                or previous_segment_id != self.request_id):
            return False
        if (type(next_segment_id) is not str
                or not 8 <= len(next_segment_id) <= 64
                or next_segment_id == previous_segment_id):
            return False
        if not stopped_independently(
                world_paused=world_paused, world_age_s=world_age_s,
                guard=guard, guard_age_s=guard_age_s,
                cmd=cmd, cmd_age_s=cmd_age_s,
                strict_v6=True,
                nominal_segment_id=previous_segment_id):
            return False

        self.phase = 'WAIT'
        self.expected_next_segment_id = next_segment_id
        self.request_id = ''
        self.kind = ''
        self.waypoint_index = -1
        self.direction = ''
        self.target = None
        self.origin = None
        self.limit = 0.0
        self.started_s = None
        self.stop_at_s = None
        self.reason = 'REARM_VERIFIED'
        return True

    def _result(self):
        return {
            'source': 'v6_geofence_watchdog', 'mode': 'AVOID',
            'armed': False,  # ROS owner must independently supply arming gate
            'segment_id': self.request_id,
            'waypoint_index': self.waypoint_index, 'kind': self.kind,
            'action': self.phase,
            'reason': self.reason,
            'ready_for_motion': False,  # deliberately never armable in this offline version
            'candidate_for_motion': self.phase == 'MONITOR',
            'motor_commands': False,
        }

    def _abort(self, reason):
        self.phase, self.reason = 'PAUSE', reason
        return self._result()

    def _coordinate_envelope(self, pose):
        x, y, yaw = pose
        x0, y0, yaw0 = self.origin
        dx, dy = x-x0, y-y0
        if self.kind == 'ADVANCE':
            along = dx*math.cos(yaw0)+dy*math.sin(yaw0)
            lateral = -dx*math.sin(yaw0)+dy*math.cos(yaw0)
            valid = all((
                -0.01 <= along <= self.limit + MAX_OVERSHOOT_M,
                abs(lateral) <= MAX_CROSS_TRACK_M,
                abs(wrap(yaw-yaw0)) <= .09,
            ))
            return valid, along >= self.limit - .012
        signed = wrap(yaw-yaw0) * (1 if self.direction == 'LEFT' else -1)
        return all((
            -MAX_OVERSHOOT_RAD <= signed <= self.limit + MAX_OVERSHOOT_RAD,
            math.hypot(dx, dy) <= MAX_CROSS_TRACK_M,
        )), signed >= self.limit - math.radians(2)

    def tick(self, *, request, request_age_s, pose, pose_age_s, now_s,
             observer, observer_age_s, guard, guard_age_s, cmd, cmd_age_s,
             world_paused, world_age_s, pause_ack=False):
        if self.phase in ('PAUSE', 'COMPLETE'):
            return self._result()
        if not finite(now_s):
            return self._abort('BAD_TIME')
        if self.phase == 'WAIT':
            if request is None:
                return self._result()
            if (self.expected_next_segment_id and
                    (not isinstance(request, dict) or
                     request.get('segment_id') != self.expected_next_segment_id)):
                return self._abort('REARM_SEGMENT_ID_MISMATCH')
            if not valid_request(request) or not recent(request_age_s, MAX_REQUEST_AGE_S):
                return self._abort('BAD_INITIAL_REQUEST')
            if not pose_valid(pose) or not recent(pose_age_s, MAX_ODOM_AGE_S):
                return self._abort('BAD_INITIAL_ODOMETRY')
            start = request['start_pose']
            if math.hypot(pose[0]-start[0], pose[1]-start[1]) > .025 \
                    or abs(wrap(pose[2]-start[2])) > .045:
                return self._abort('START_POSE_MISMATCH')
            if not observer_healthy(observer, observer_age_s) or not (
                    front_clear(observer) if request['kind'] == 'ADVANCE'
                    else turn_clear(observer)):
                return self._abort('START_SENSORS_OR_CLEARANCE')
            if not stopped_independently(world_paused=world_paused,
                                         world_age_s=world_age_s,
                                         guard=guard, guard_age_s=guard_age_s,
                                         cmd=cmd, cmd_age_s=cmd_age_s,
                                         strict_v6=self.strict_v6_guard):
                return self._abort('START_NOT_STOPPED')
            self.request_id = request['segment_id']
            self.waypoint_index = request['waypoint_index']
            self.kind = request['kind']
            self.direction = request.get('direction') or ''
            self.target = tuple(request['target'])
            self.origin = tuple(start)
            self.limit = float(request['limit'])
            self.started_s = now_s
            self.phase, self.reason = 'MONITOR', 'WITHIN_SEGMENT_ENVELOPE'
            return self._result()
        if self.phase == 'MONITOR':
            if not recent(request_age_s, MAX_REQUEST_AGE_S) or not isinstance(request, dict) \
                    or request.get('segment_id') != self.request_id:
                return self._abort('REQUEST_LOST_OR_CHANGED')
            if not pose_valid(pose) or not recent(pose_age_s, MAX_ODOM_AGE_S):
                return self._abort('ODOMETRY_STALE')
            if not observer_healthy(observer, observer_age_s):
                return self._abort('OBSERVER_STALE')
            if not (front_clear(observer) if self.kind == 'ADVANCE'
                    else turn_clear(observer)):
                return self._abort('OBSTACLE_CLEARANCE_LOST')
            if not 0 <= now_s-self.started_s <= MAX_SEGMENT_WALL_S:
                return self._abort('SEGMENT_TIMEOUT')
            in_bounds, reached = self._coordinate_envelope(pose)
            if not in_bounds:
                return self._abort('SEGMENT_ENVELOPE_BREACHED')
            if request.get('kind') != self.kind or request.get('waypoint_index') != self.waypoint_index \
                    or request.get('direction', '') != self.direction \
                    or not finite(request.get('limit')) \
                    or abs(request['limit'] - self.limit) > 1e-9 \
                    or not pose_valid(request.get('start_pose')) \
                    or tuple(request['start_pose']) != self.origin \
                    or not isinstance(request.get('target'), (tuple, list)) \
                    or tuple(request['target']) != self.target:
                return self._abort('REQUEST_MUTATED')
            if request.get('phase') == 'HOLD':
                if not valid_request(request, moving=False) or not reached:
                    return self._abort('EARLY_OR_BAD_COMPLETION')
                self.phase, self.reason, self.stop_at_s = 'STOPPING', 'TARGET_REACHED', now_s
                return self._result()
            if not valid_request(request) or request.get('kind') != self.kind \
                    or request.get('waypoint_index') != self.waypoint_index \
                    or request.get('direction', '') != self.direction \
                    or abs(request['limit']-self.limit) > 1e-9 \
                    or tuple(request['start_pose']) != self.origin:
                return self._abort('REQUEST_MUTATED')
            if reached:
                # Never allow further forward/turn velocity once reaching target.
                self.phase, self.reason, self.stop_at_s = 'STOPPING', 'TARGET_REACHED', now_s
            return self._result()
        if self.phase == 'STOPPING':
            if not recent(now_s-self.stop_at_s, MAX_STOP_WALL_S):
                return self._abort('STOP_TIMEOUT')
            if not isinstance(request, dict) or request.get('segment_id') != self.request_id \
                    or not valid_request(request, moving=False) or not recent(request_age_s, MAX_REQUEST_AGE_S) \
                    or request.get('kind') != self.kind \
                    or request.get('waypoint_index') != self.waypoint_index \
                    or not finite(request.get('limit')) or abs(request['limit']-self.limit) > 1e-9 \
                    or not pose_valid(request.get('start_pose')) or tuple(request['start_pose']) != self.origin \
                    or not isinstance(request.get('target'), (tuple, list)) \
                    or tuple(request['target']) != self.target:
                return self._abort('STOP_REQUEST_MISSING')
            if not pose_valid(pose) or not recent(pose_age_s, MAX_ODOM_AGE_S):
                return self._abort('STOP_POSE_MISSING')
            in_bounds, reached = self._coordinate_envelope(pose)
            if not in_bounds or not reached:
                return self._abort('STOP_POSITION_INVALID')
            if pause_ack is True and stopped_independently(
                    world_paused=world_paused, world_age_s=world_age_s,
                    guard=guard, guard_age_s=guard_age_s, cmd=cmd, cmd_age_s=cmd_age_s,
                    strict_v6=self.strict_v6_guard,
                    nominal_segment_id=(self.request_id if self.strict_v6_guard else None)):
                self.phase, self.reason = 'COMPLETE', 'TARGET_REACHED'
            return self._result()
        return self._abort('INVALID_INTERNAL_PHASE')
