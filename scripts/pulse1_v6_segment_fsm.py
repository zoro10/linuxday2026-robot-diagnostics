#!/usr/bin/env python3
"""PULSE-1 V6 segment execution core: independent STOP acknowledgments.

Pure Python logic, deliberately has NO ROS publisher or motor interface.
The V6 ROS executor / V6 Motor Guard / V6 safety watchdog are not yet installed.
Never use this class as an authorization to move the rover.
"""

from dataclasses import dataclass
import math

MAX_TRANSLATION_M = 0.20
MAX_ROTATION_RAD = math.radians(15)
MAX_START_POSE_ERROR_M = 0.025
MAX_START_YAW_ERROR_RAD = 0.045
MAX_SEGMENT_WALL_S = 45.0
MAX_STOP_WALL_S = 4.0
MAX_OVERSHOOT_M = 0.015
MAX_OVERSHOOT_RAD = math.radians(2)
MIN_PROGRESS_M = 0.005
MIN_PROGRESS_RAD = math.radians(0.5)
FRESH_S = 0.6


def finite(v):
    return type(v) in (float, int) and math.isfinite(v)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_valid(p):
    return isinstance(p, (tuple, list)) and len(p) == 3 and all(finite(v) for v in p)


def stopped_evidence(*, paused, paused_age_s, guard, guard_age_s,
                     cmd, cmd_age_s):
    return all((
        paused is True,
        finite(paused_age_s) and 0 <= paused_age_s <= FRESH_S,
        isinstance(guard, dict) and guard.get('source') == 'motor_guard_live'
        and guard.get('mode') == 'AVOID' and guard.get('motor_commands') is True
        and guard.get('guard_state') in ('BLOCKED', 'STALE_REQUEST')
        and guard.get('permitted_linear_m_s') == 0
        and guard.get('permitted_angular_rad_s') == 0,
        finite(guard_age_s) and 0 <= guard_age_s <= FRESH_S,
        isinstance(cmd, (list, tuple)) and len(cmd) == 2
        and all(finite(v) and abs(v) <= 1e-8 for v in cmd),
        finite(cmd_age_s) and 0 <= cmd_age_s <= FRESH_S,
    ))


@dataclass(frozen=True)
class Segment:
    waypoint_index: int
    kind: str
    start_pose: tuple
    target: tuple
    limit: float
    direction: str = ''

    @staticmethod
    def from_shadow(status):
        if not isinstance(status, dict) or any((
            status.get('source') != 'waypoint_segment_shadow',
            status.get('mode') != 'AVOID',
            status.get('state') != 'CANDIDATE',
            status.get('reason') != 'OFFLINE_PROPOSAL_ONLY',
            status.get('motion_authorized') is not False,
            status.get('motor_commands') is not False,
            status.get('proposed_linear_m_s') != 0,
            status.get('proposed_angular_rad_s') != 0,
        )):
            raise ValueError('INVALID_SHADOW_CANDIDATE')
        c = status.get('candidate')
        if not isinstance(c, dict) or c.get('requires_independent_stop_ack') is not True:
            raise ValueError('STOP_ACK_NOT_REQUIRED')
        i, start, target = c.get('waypoint_index'), c.get('start_pose'), c.get('target')
        if type(i) is not int or i not in range(5) or not pose_valid(start) \
                or not isinstance(target, (tuple, list)) or len(target) != 2 \
                or not all(finite(v) for v in target):
            raise ValueError('INVALID_SEGMENT_GEOMETRY')
        kind = c.get('kind')
        if kind == 'ADVANCE':
            lim = c.get('max_distance_m')
            if not finite(lim) or not 0 < lim <= MAX_TRANSLATION_M:
                raise ValueError('FORWARD_CAP_EXCEEDED')
            direction = ''
        elif kind == 'TURN':
            lim, direction = c.get('max_angle_rad'), c.get('direction')
            if not finite(lim) or not 0 < lim <= round(MAX_ROTATION_RAD, 6) \
                    or direction not in ('LEFT', 'RIGHT'):
                raise ValueError('TURN_CAP_EXCEEDED')
        else:
            raise ValueError('UNKNOWN_SEGMENT_KIND')
        return Segment(i, kind, tuple(start), tuple(target), float(lim), direction)

    @staticmethod
    def from_executor_request(request):
        if not isinstance(request, dict) or any((
            request.get('source') != 'v6_segment_executor',
            request.get('mode') != 'AVOID',
            request.get('phase') != 'DRIVE',
            request.get('motor_commands') is not False,
            not isinstance(request.get('segment_id'), str)
            or not request.get('segment_id'),
        )):
            raise ValueError('INVALID_EXECUTOR_REQUEST')

        i = request.get('waypoint_index')
        start = request.get('start_pose')
        target = request.get('target')
        kind = request.get('kind')
        limit = request.get('limit')
        direction = request.get('direction', '')
        linear = request.get('proposed_linear_m_s')
        angular = request.get('proposed_angular_rad_s')

        if (type(i) is not int or i not in range(5)
                or not pose_valid(start)
                or not isinstance(target, (tuple, list))
                or len(target) != 2
                or not all(finite(v) for v in target)):
            raise ValueError('INVALID_SEGMENT_GEOMETRY')
        if not finite(linear) or not finite(angular):
            raise ValueError('INVALID_EXECUTOR_COMMAND')

        if kind == 'ADVANCE':
            if not finite(limit) or not 0 < limit <= MAX_TRANSLATION_M:
                raise ValueError('FORWARD_CAP_EXCEEDED')
            if linear <= 0 or abs(angular) > 1e-8:
                raise ValueError('INVALID_EXECUTOR_COMMAND')
            direction = ''
        elif kind == 'TURN':
            if (not finite(limit) or not 0 < limit <= round(MAX_ROTATION_RAD, 6)
                    or direction not in ('LEFT', 'RIGHT')):
                raise ValueError('TURN_CAP_EXCEEDED')
            if abs(linear) > 1e-8 or abs(angular) <= 1e-8:
                raise ValueError('INVALID_EXECUTOR_COMMAND')
        else:
            raise ValueError('UNKNOWN_SEGMENT_KIND')

        return Segment(i, kind, tuple(start), tuple(target),
                       float(limit), direction)


class V6SegmentGate:
    """Conservative execution accounting for ONE segment at a time.

    This gate records a candidate and evaluates STOP. It does NOT grant motor
    permission, create requests, or step physics. Motion cannot start without
    an independent V6 motor authorization and V6 geofence (future components).
    """
    def __init__(self):
        self.phase = 'IDLE'
        self.current = None
        self.started_at = None
        self.stop_requested_at = None
        self.last_end_pose = None
        self.abort_reason = None
        self.ack_count = 0

    def _abort(self, reason):
        self.phase = 'ABORT'
        self.abort_reason = reason
        return self.phase

    def prepare(self, status, actual_pose, *, paused, paused_age_s,
                guard, guard_age_s, cmd, cmd_age_s):
        if self.phase != 'IDLE':
            return self._abort('UNEXPECTED_PREPARE')
        try:
            seg = Segment.from_shadow(status)
        except ValueError as exc:
            return self._abort(str(exc))
        if not pose_valid(actual_pose):
            return self._abort('POSE_UNAVAILABLE')
        if not stopped_evidence(paused=paused, paused_age_s=paused_age_s,
                                guard=guard, guard_age_s=guard_age_s,
                                cmd=cmd, cmd_age_s=cmd_age_s):
            return self._abort('STOP_NOT_PROVEN')
        if math.hypot(actual_pose[0] - seg.start_pose[0],
                      actual_pose[1] - seg.start_pose[1]) > MAX_START_POSE_ERROR_M \
           or abs(wrap(actual_pose[2] - seg.start_pose[2])) > MAX_START_YAW_ERROR_RAD:
            return self._abort('START_POSE_DISAGREE')
        if self.last_end_pose is not None and (math.hypot(
                actual_pose[0] - self.last_end_pose[0],
                actual_pose[1] - self.last_end_pose[1]) > MAX_START_POSE_ERROR_M
                or abs(wrap(actual_pose[2] - self.last_end_pose[2])) > MAX_START_YAW_ERROR_RAD):
            return self._abort('CHECKPOINT_REPLAY_OR_JUMP')
        self.current = seg
        self.phase = 'PREPARED'
        return self.phase

    def prepare_executor_request(self, request, actual_pose, *,
                                 paused, paused_age_s, guard, guard_age_s,
                                 cmd, cmd_age_s):
        if self.phase != 'IDLE':
            return self._abort('UNEXPECTED_PREPARE')
        try:
            seg = Segment.from_executor_request(request)
        except ValueError as exc:
            return self._abort(str(exc))
        if not pose_valid(actual_pose):
            return self._abort('POSE_UNAVAILABLE')
        if not stopped_evidence(paused=paused, paused_age_s=paused_age_s,
                                guard=guard, guard_age_s=guard_age_s,
                                cmd=cmd, cmd_age_s=cmd_age_s):
            return self._abort('STOP_NOT_PROVEN')
        if (math.hypot(actual_pose[0] - seg.start_pose[0],
                       actual_pose[1] - seg.start_pose[1]) > MAX_START_POSE_ERROR_M
                or abs(wrap(actual_pose[2] - seg.start_pose[2])
                       ) > MAX_START_YAW_ERROR_RAD):
            return self._abort('START_POSE_DISAGREE')
        if self.last_end_pose is not None and (
                math.hypot(actual_pose[0] - self.last_end_pose[0],
                           actual_pose[1] - self.last_end_pose[1]) > MAX_START_POSE_ERROR_M
                or abs(wrap(actual_pose[2] - self.last_end_pose[2])) > MAX_START_YAW_ERROR_RAD):
            return self._abort('CHECKPOINT_REPLAY_OR_JUMP')
        self.current = seg
        self.phase = 'PREPARED'
        return self.phase

    def record_authorized_motion(self, *, now_s, v6_watchdog_confirmed,
                                 v6_motor_authorized):
        """To be called only by future independent V6 components, not by UI."""
        if self.phase != 'PREPARED':
            return self._abort('UNEXPECTED_MOTION')
        if v6_watchdog_confirmed is not True or v6_motor_authorized is not True \
                or not finite(now_s):
            return self._abort('V6_AUTHORIZATION_MISSING')
        self.started_at = now_s
        self.phase = 'ACTIVE'
        return self.phase

    def progress(self, pose, *, now_s, pose_age_s, reached_tolerance_m=0.008):
        if self.phase != 'ACTIVE':
            return self._abort('UNEXPECTED_PROGRESS')
        if (not finite(reached_tolerance_m)
                or not 0 <= reached_tolerance_m <= 0.015):
            return self._abort('INVALID_REACHED_TOLERANCE')
        if not pose_valid(pose) or not finite(pose_age_s) or not 0 <= pose_age_s <= FRESH_S \
                or not finite(now_s) or not 0 <= now_s - self.started_at <= MAX_SEGMENT_WALL_S:
            return self._abort('POSE_OR_TIME_INVALID')
        seg = self.current
        x0,y0,yaw0 = seg.start_pose
        dx,dy = pose[0]-x0, pose[1]-y0
        if seg.kind == 'ADVANCE':
            along = dx*math.cos(yaw0)+dy*math.sin(yaw0)
            cross = -dx*math.sin(yaw0)+dy*math.cos(yaw0)
            if along < -0.01 or along > seg.limit + MAX_OVERSHOOT_M \
                    or abs(cross) > 0.035 or abs(wrap(pose[2]-yaw0)) > 0.09:
                return self._abort('FORWARD_ENVELOPE_BREACHED')
            reached = along >= max(MIN_PROGRESS_M, seg.limit - reached_tolerance_m)
        else:
            yaw_change = wrap(pose[2]-yaw0)
            progress = yaw_change if seg.direction == 'LEFT' else -yaw_change
            if progress < -MAX_OVERSHOOT_RAD or progress > seg.limit + MAX_OVERSHOOT_RAD \
                    or math.hypot(dx,dy) > 0.035:
                return self._abort('TURN_ENVELOPE_BREACHED')
            reached = progress >= max(MIN_PROGRESS_RAD, seg.limit - math.radians(1.5))
        if reached:
            self.phase = 'STOP_REQUESTED'
            self.stop_requested_at = now_s
        return self.phase

    def acknowledge_stop(self, pose, *, now_s, pose_age_s, world_step_ack, paused, paused_age_s,
                         guard, guard_age_s, cmd, cmd_age_s,
                         independent_stop_ack):
        if self.phase != 'STOP_REQUESTED':
            return self._abort('STOP_OUT_OF_SEQUENCE')
        if independent_stop_ack is not True or world_step_ack is not True:
            return self._abort('NO_INDEPENDENT_STOP_ACK')
        if not finite(now_s) or not 0 <= now_s - self.stop_requested_at <= MAX_STOP_WALL_S:
            return self._abort('STOP_TIMEOUT')
        if not stopped_evidence(paused=paused, paused_age_s=paused_age_s,
                                guard=guard, guard_age_s=guard_age_s,
                                cmd=cmd, cmd_age_s=cmd_age_s):
            return self._abort('NOT_STOPPED')
        if not pose_valid(pose) or not finite(pose_age_s) or not 0 <= pose_age_s <= FRESH_S:
            return self._abort('FINAL_POSE_STALE_OR_INVALID')
        seg=self.current
        x0,y0,yaw0=seg.start_pose
        if seg.kind == 'ADVANCE':
            dx,dy=pose[0]-x0,pose[1]-y0
            along=dx*math.cos(yaw0)+dy*math.sin(yaw0)
            cross=-dx*math.sin(yaw0)+dy*math.cos(yaw0)
            correct=(seg.limit-0.012 <= along <= seg.limit+MAX_OVERSHOOT_M
                     and abs(cross)<=0.035 and abs(wrap(pose[2]-yaw0))<=0.09)
        else:
            signed=wrap(pose[2]-yaw0)*(1 if seg.direction=='LEFT' else -1)
            correct=(seg.limit-math.radians(2.0)<=signed<=seg.limit+MAX_OVERSHOOT_RAD
                     and math.hypot(pose[0]-x0,pose[1]-y0)<=0.035)
        if not correct:
            return self._abort('CHECKPOINT_DISTANCE_INVALID')
        self.last_end_pose=tuple(pose)
        self.ack_count+=1
        self.phase='STOP_CONFIRMED'
        return self.phase

    def next_segment(self):
        if self.phase != 'STOP_CONFIRMED':
            return self._abort('NEXT_WITHOUT_STOP')
        self.current=None
        self.started_at=None
        self.stop_requested_at=None
        self.phase='IDLE'
        return self.phase
