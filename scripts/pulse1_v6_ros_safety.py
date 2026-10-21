#!/usr/bin/env python3
"""PULSE-1 V6 ROS 2 safety nodes: Geofence watchdog and exclusive motor guard.

Default mode: DISARMED. Never start two motor publishers.
These are infrastructure nodes, not a segment executor. In particular they
NEVER publish a V6 motion request. No autonomous motion is implemented here.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import WorldStatistics
from ros_gz_interfaces.srv import ControlWorld

from pulse1_v6_safety_contract import (
    V6GeofenceSegment, v6_guard_velocity, valid_request, pose_valid,
    stopped_independently,
)
from pulse1_v6_ros_safety_policy import watchdog_lease, motor_guard_status, nominal_v6_stop
from pulse1_v6_rearm_policy import verify_rearm

ROOT = Path(__file__).resolve().parent.parent
CMD_TOPIC = '/model/pulse1_proto/cmd_vel'
REQUEST_TOPIC = '/pulse1/v6_segment_request'
WATCHDOG_TOPIC = '/pulse1/v6_geofence_watchdog'
GUARD_TOPIC = '/pulse1/v6_motor_guard'


def scenario_s1_avoid():
    try:
        obj = json.loads((ROOT / 'runtime/pulse1_world.json').read_text())
        return (obj.get('scenario') == 's1' and
                obj.get('navigation_mode') == 'AVOID' and
                obj.get('world') == 'empty')
    except (OSError, ValueError, TypeError):
        return False



def locked_segment_from_checkpoint(snapshot):
    """Legge evidenza storica; NON autorizza movimento."""
    if not isinstance(snapshot, dict):
        raise ValueError('INVALID_CHECKPOINT')

    required = (
        'GAZEBO_PAUSED',
        'GUARD_NOMINAL_LATCHED',
        'GEOFENCE_COMPLETE',
        'SEGMENT_ID_MATCH',
        'CMD_ZERO',
        'GUARD_EXCLUSIVE',
        'EXECUTOR_ASSENTE',
        'SUPERVISOR_HEALTHY',
    )

    checks = snapshot.get('checks')
    guard = snapshot.get('guard')
    gf = snapshot.get('geofence')
    sid = snapshot.get('segment_id')
    pose = snapshot.get('pose')
    cmd = snapshot.get('cmd_vel')

    if not (
        snapshot.get('record_type') == 'HISTORICAL_STOP_EVIDENCE'
        and snapshot.get('motion_authorized') is False
        and snapshot.get('world_paused') is True
        and snapshot.get('position_source') == 'GAZEBO_MODEL_STATE'
        and isinstance(checks, dict)
        and all(checks.get(k) is True for k in required)
        and isinstance(guard, dict)
        and isinstance(gf, dict)
        and isinstance(sid, str)
        and 8 <= len(sid) <= 64
        and guard.get('segment_id') == sid
        and gf.get('segment_id') == sid
        and guard.get('nominal_stop_latched') is True
        and guard.get('emergency_latched') is False
        and gf.get('action') == 'COMPLETE'
        and gf.get('reason') == 'TARGET_REACHED'
        and isinstance(pose, list)
        and len(pose) == 3
        and all(type(v) in (int, float) and math.isfinite(v)
                for v in pose)
        and isinstance(cmd, list)
        and len(cmd) == 2
        and all(type(v) in (int, float) and math.isfinite(v)
                and abs(v) <= 1e-8 for v in cmd)
    ):
        raise ValueError('INVALID_STOP_EVIDENCE')

    if not (
        0.05 <= pose[0] <= 0.14
        and abs(pose[1]) <= 0.04
        and abs(pose[2]) <= 0.09
    ):
        raise ValueError('INVALID_CHECKPOINT_POSE')

    return sid


def restore_locked_node(node, role, segment_id):
    """Ripristino esclusivamente bloccato, prima dello spin ROS."""
    if not node.armed:
        raise RuntimeError('RESTORE_REQUIRES_ARMED_NODE')

    if role == 'guard':
        if node.emergency_latched:
            raise RuntimeError('EMERGENCY_LATCH_PRESENT')
        node.nominal_stop_latched = True
        node.active_segment_id = segment_id
        node.expected_next_segment_id = None
        node.last_active = False

    elif role == 'geofence':
        node.core.phase = 'COMPLETE'
        node.core.reason = 'TARGET_REACHED'
        node.core.request_id = segment_id
        node.core.expected_next_segment_id = ''

    else:
        raise RuntimeError('INVALID_RESTORE_ROLE')

    node.restore_pause_pending = True


def pose_from_odom(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v = [p.x, p.y, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(x) for x in v):
        return None
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if not .5 <= norm <= 1.5:
        return None
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return (float(p.x), float(p.y), yaw)


class Inputs(Node):
    def __init__(self, name):
        super().__init__(name)
        self.values = {}
        self.times = {}

    def store(self, field, value):
        self.values[field] = value
        self.times[field] = time.monotonic()

    def age(self, field, now):
        stamp = self.times.get(field)
        return now - stamp if stamp is not None else None

    def on_json(self, field, msg):
        try:
            value = json.loads(msg.data)
            self.store(field, value if isinstance(value, dict) else None)
        except (ValueError, TypeError):
            self.store(field, None)

    def on_pose(self, msg):
        self.store('pose', pose_from_odom(msg))

    def on_cmd(self, msg):
        self.store('cmd', (msg.linear.x, msg.angular.z))

    def on_stats(self, msg):
        self.store('paused', msg.paused)

    def add_input(self, field, topic):
        self.create_subscription(
            String, topic, lambda msg, f=field: self.on_json(f, msg), 10)

    def add_pose(self):
        self.create_subscription(Odometry, '/model/pulse1_proto/odometry',
                                 self.on_pose, qos_profile_sensor_data)

    def add_cmd(self):
        self.create_subscription(Twist, CMD_TOPIC, self.on_cmd, 10)

    def add_stats(self):
        self.create_subscription(WorldStatistics, '/world/empty/stats',
                                 self.on_stats, qos_profile_sensor_data)


class EmergencyPauser:
    """ROS async PAUSE: no blocking calls inside ROS timer callbacks."""
    def setup_pause(self):
        self.pause_client = self.create_client(ControlWorld, '/world/empty/control')
        self.pause_future = None
        self.pause_sent_at = None
        self.pause_ack = False
        self.pause_error = None

    def start_pause(self):
        if self.pause_future is not None or self.pause_ack or self.pause_error:
            return
        if not self.pause_client.service_is_ready():
            self.pause_error = 'CONTROL_WORLD_UNAVAILABLE'
            return
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.multi_step = 0
        try:
            self.pause_future = self.pause_client.call_async(req)
            self.pause_sent_at = time.monotonic()
        except Exception:
            self.pause_error = 'PAUSE_CALL_FAILED'

    def poll_pause(self):
        if getattr(self, 'restore_pause_pending', False):
            if self.pause_error is None and self.pause_client.service_is_ready():
                self.start_pause()
                if self.pause_future is not None or self.pause_ack:
                    self.restore_pause_pending = False
        if self.pause_future is None:
            return
        if self.pause_future.done():
            try:
                response = self.pause_future.result()
                self.pause_ack = bool(response and response.success is True)
            except Exception:
                self.pause_ack = False
            if not self.pause_ack:
                self.pause_error = 'PAUSE_ACK_FAILED'
            self.pause_future = None
        elif time.monotonic() - self.pause_sent_at > 3.0:
            self.pause_error = 'PAUSE_TIMEOUT'
            # A timed out pause request might still be running: DO NOT resend.
            self.pause_future = None


class V6Geofence(Inputs, EmergencyPauser):
    def __init__(self, armed):
        super().__init__('pulse1_v6_geofence_watchdog')
        self.armed = armed
        self.core = V6GeofenceSegment(strict_v6_guard=True)
        self.pending_rearm = None
        self.rearm_previous_id = None
        self.setup_pause()
        self.add_input('request', REQUEST_TOPIC)
        self.add_input('observer', '/pulse1/avoid_observer')
        self.add_input('guard', GUARD_TOPIC)
        self.add_pose()
        self.add_cmd()
        self.add_stats()
        self.create_subscription(
            String, '/pulse1/v6_geofence_rearm',
            self.receive_rearm, 10)
        self.publisher = self.create_publisher(String, WATCHDOG_TOPIC, 10)
        self.create_timer(.05, self.control)


    def receive_rearm(self, msg):
        try:
            request = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(request, dict):
            self.pending_rearm = request

    def process_rearm(self, kw):
        request = self.pending_rearm
        self.pending_rearm = None

        if request is None:
            return False

        # Riarmo manuale abilitato esplicitamente all'avvio del nodo.
        if os.environ.get('PULSE1_V6_REARM_ENABLE') != 'YES':
            self.get_logger().warning('REARM DENIED: OPERATOR_GATE')
            return False

        # Nessun executor deve ancora pubblicare richieste.
        if self.get_publishers_info_by_topic(REQUEST_TOPIC):
            self.get_logger().warning('REARM DENIED: EXECUTOR_PRESENT')
            return False

        if self.pause_error or self.pause_future is not None:
            self.get_logger().warning('REARM DENIED: PAUSE_NOT_SETTLED')
            return False

        guard = kw['guard']
        if (not isinstance(guard, dict)
                or guard.get('rearm_prepared_for') !=
                    request.get('next_segment_id')
                or guard.get('rearm_previous_segment_id') !=
                    request.get('previous_segment_id')):
            self.get_logger().warning('REARM DENIED: GUARD_NOT_PREPARED')
            return False

        accepted = self.core.rearm_after_confirmed_stop(
            previous_segment_id=request.get('previous_segment_id'),
            next_segment_id=request.get('next_segment_id'),
            operator_authorized=(
                request.get('operator_authorized') is True),
            previous_executor_exited=True,
            pause_ack=self.pause_ack,
            guard=kw['guard'],
            guard_age_s=kw['guard_age_s'],
            cmd=kw['cmd'],
            cmd_age_s=kw['cmd_age_s'],
            world_paused=kw['world_paused'],
            world_age_s=kw['world_age_s'],
        )

        if not accepted:
            self.get_logger().warning('REARM DENIED: STOP_PROOF')
            return False

        self.rearm_previous_id = request['previous_segment_id']

        # La conferma dello STOP precedente NON è valida per
        # un futuro segmento: servirà un nuovo PAUSE/ACK.
        self.pause_ack = False
        self.pause_sent_at = None

        # Elimina anche l'ultima richiesta memorizzata.
        self.values.pop('request', None)
        self.times.pop('request', None)
        kw['request'] = None
        kw['request_age_s'] = None

        self.get_logger().info(
            'GEOFENCE REARM PREPARED: ' +
            self.core.expected_next_segment_id
        )
        return True

    def control(self):
        now = time.monotonic()
        self.poll_pause()
        kw = dict(
            request=self.values.get('request'), request_age_s=self.age('request', now),
            pose=self.values.get('pose'), pose_age_s=self.age('pose', now), now_s=now,
            observer=self.values.get('observer'), observer_age_s=self.age('observer', now),
            guard=self.values.get('guard'), guard_age_s=self.age('guard', now),
            cmd=self.values.get('cmd'), cmd_age_s=self.age('cmd', now),
            world_paused=self.values.get('paused'), world_age_s=self.age('paused', now),
            pause_ack=self.pause_ack,
        )
        # Do not interpret a request when unarmed; remain in WAIT.
        if not self.armed:
            kw['request'] = None
            kw['request_age_s'] = None
        self.process_rearm(kw)
        result = self.core.tick(**kw)
        if self.pause_error:
            result = self.core._abort(self.pause_error)
        if self.armed and result['action'] in ('STOPPING', 'PAUSE'):
            self.start_pause()
        lease = watchdog_lease(
            result=result, arm_enabled=self.armed,
            request=kw['request'], request_age_s=kw['request_age_s'],
            observer=kw['observer'], observer_age_s=kw['observer_age_s'],
            pose=kw['pose'], pose_age_s=kw['pose_age_s'],
            guard=kw['guard'], guard_age_s=kw['guard_age_s'],
            cmd_age_s=kw['cmd_age_s'], world_age_s=kw['world_age_s'],
            pause_service_ready=self.pause_client.service_is_ready(),
        )
        status = {**result, 'armed': self.armed,
                  'ready_for_motion': lease,
                  'pause_ack': self.pause_ack, 'pause_error': self.pause_error,
                  'motor_commands': False}
        status['rearm_previous_segment_id'] = self.rearm_previous_id
        status['rearm_next_segment_id'] = (
            self.core.expected_next_segment_id or None
        )
        msg = String()
        msg.data = json.dumps(status, allow_nan=False)
        self.publisher.publish(msg)


class V6MotorGuard(Inputs, EmergencyPauser):
    def __init__(self, armed):
        super().__init__('pulse1_v6_motor_guard')
        self.armed = armed
        self.last_active = False
        self.emergency_latched = False
        self.nominal_stop_latched = False
        self.active_segment_id = None
        self.expected_next_segment_id = None
        self.pending_rearm_prepare = None
        self.rearm_prepared = None
        self.setup_pause()
        self.add_input('request', REQUEST_TOPIC)
        self.add_input('watchdog', WATCHDOG_TOPIC)
        self.add_input('observer', '/pulse1/avoid_observer')
        self.add_pose()
        self.add_cmd()
        self.add_stats()
        self.create_subscription(
            String, '/pulse1/v6_guard_rearm_prepare',
            self.receive_rearm_prepare, 10)
        self.guard_output = self.create_publisher(String, GUARD_TOPIC, 10)
        self.motor = None
        if armed:
            # Fail if V5 or any other controller already owns cmd_vel.
            if self.get_publishers_info_by_topic(CMD_TOPIC):
                raise RuntimeError('CMD_VEL_NOT_EXCLUSIVE: stop V5 controller first')
            if not self.pause_client.wait_for_service(timeout_sec=8.0):
                raise RuntimeError('CONTROL_WORLD_UNAVAILABLE')
            self.motor = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_timer(.05, self.control)


    def receive_rearm_prepare(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            self.pending_rearm_prepare = data

    def guard_stop_snapshot(self):
        exclusive = (
            self.motor is not None
            and len(self.get_publishers_info_by_topic(CMD_TOPIC)) == 1
        )
        status = motor_guard_status(
            armed=self.armed, velocity=(0.0, 0.0),
            request=None,
            pause_ready=(
                self.pause_client.service_is_ready()
                and not self.pause_error),
            exclusive=exclusive,
        )
        status['emergency_latched'] = self.emergency_latched
        status['nominal_stop_latched'] = self.nominal_stop_latched
        status['segment_id'] = self.active_segment_id
        return status

    def process_guard_rearm_protocol(self, now):
        proposal = self.pending_rearm_prepare
        self.pending_rearm_prepare = None

        if self.rearm_prepared is not None:
            old, new, when = self.rearm_prepared
            if (now - when > 30.0 or self.emergency_latched
                    or self.pause_error):
                self.rearm_prepared = None

        if proposal is not None:
            ready = all((
                os.environ.get('PULSE1_V6_REARM_ENABLE') == 'YES',
                self.armed,
                self.nominal_stop_latched,
                not self.emergency_latched,
                not self.last_active,
                self.pause_ack,
                self.pause_error is None,
                self.pause_future is None,
                scenario_s1_avoid(),
                not self.get_publishers_info_by_topic(REQUEST_TOPIC),
            ))

            if ready and proposal.get('previous_segment_id') == self.active_segment_id:
                allowed = verify_rearm(
                    previous_id=proposal.get('previous_segment_id'),
                    next_id=proposal.get('next_segment_id'),
                    operator_authorized=(
                        proposal.get('operator_authorized') is True),
                    previous_executor_exited=True,
                    watchdog=self.values.get('watchdog'),
                    watchdog_age_s=self.age('watchdog', now),
                    guard=self.guard_stop_snapshot(),
                    guard_age_s=0.0,
                    cmd=self.values.get('cmd'),
                    cmd_age_s=self.age('cmd', now),
                    world_paused=self.values.get('paused'),
                    world_age_s=self.age('paused', now),
                )
                if allowed:
                    self.rearm_prepared = (
                        proposal['previous_segment_id'],
                        proposal['next_segment_id'],
                        now,
                    )
                    self.get_logger().info('GUARD REARM PREPARED')
                else:
                    self.get_logger().warning('GUARD REARM PROOF DENIED')
            else:
                self.get_logger().warning('GUARD REARM PREPARE DENIED')

        if self.rearm_prepared is None:
            return

        old, new, _ = self.rearm_prepared
        watchdog = self.values.get('watchdog')

        if not isinstance(watchdog, dict):
            return
        if watchdog.get('action') == 'PAUSE':
            self.rearm_prepared = None
            return

        if not all((
            watchdog.get('source') == 'v6_geofence_watchdog',
            watchdog.get('armed') is True,
            watchdog.get('action') == 'WAIT',
            watchdog.get('reason') == 'REARM_VERIFIED',
            watchdog.get('rearm_previous_segment_id') == old,
            watchdog.get('rearm_next_segment_id') == new,
            watchdog.get('pause_error') is None,
            watchdog.get('ready_for_motion') is False,
            self.age('watchdog', now) is not None,
            0 <= self.age('watchdog', now) <= 0.6,
            self.pause_ack,
            self.pause_future is None,
            self.pause_error is None,
            not self.emergency_latched,
            not self.last_active,
            not self.get_publishers_info_by_topic(REQUEST_TOPIC),
        )):
            return

        if not stopped_independently(
                world_paused=self.values.get('paused'),
                world_age_s=self.age('paused', now),
                guard=self.guard_stop_snapshot(),
                guard_age_s=0.0,
                cmd=self.values.get('cmd'),
                cmd_age_s=self.age('cmd', now),
                strict_v6=True, nominal_segment_id=old):
            return

        self.nominal_stop_latched = False
        self.active_segment_id = None
        self.last_active = False
        self.expected_next_segment_id = new
        self.rearm_prepared = None
        self.values.pop('request', None)
        self.times.pop('request', None)
        self.pause_ack = False
        self.pause_sent_at = None
        self.get_logger().info('GUARD REARM COMMITTED')

    def control(self):
        now = time.monotonic()
        self.poll_pause()
        self.process_guard_rearm_protocol(now)
        exclusive = (self.motor is not None and
                     len(self.get_publishers_info_by_topic(CMD_TOPIC)) == 1)
        request = self.values.get('request')
        velocity = (0.0, 0.0)
        if (self.expected_next_segment_id is not None
                and request is not None
                and (not isinstance(request, dict)
                     or request.get('segment_id') !=
                         self.expected_next_segment_id)):
            self.emergency_latched = True
            self.start_pause()
        if self.armed and not exclusive:
            self.emergency_latched = True
            self.start_pause()
        if (self.armed and exclusive and not self.emergency_latched
                and not self.nominal_stop_latched
                and not self.pause_error and self.pause_client.service_is_ready()):
            velocity = v6_guard_velocity(
                request, request_age_s=self.age('request', now),
                watchdog=self.values.get('watchdog'),
                watchdog_age_s=self.age('watchdog', now),
                observer=self.values.get('observer'),
                observer_age_s=self.age('observer', now),
                odom_age_s=self.age('pose', now),
            )
        active = velocity != (0.0, 0.0)
        if active:
            # A new request may never replace the ID of a running segment.
            if self.active_segment_id is not None and (
                    not isinstance(request, dict) or
                    request.get('segment_id') != self.active_segment_id):
                self.emergency_latched = True
                self.start_pause()
                active = False
                velocity = (0.0, 0.0)
            else:
                self.active_segment_id = request['segment_id']
        if self.last_active and not active:
            # On a valid target reached, stop permanently (for this node
            # lifetime), without presenting it as a motor-fault abort.
            # ANY other authorization loss stays an emergency.
            if nominal_v6_stop(
                    watchdog=self.values.get('watchdog'),
                    watchdog_age_s=self.age('watchdog', now),
                    request=request, active_segment_id=self.active_segment_id):
                self.nominal_stop_latched = True
            else:
                self.emergency_latched = True
            self.start_pause()
        if self.emergency_latched or self.nominal_stop_latched:
            velocity = (0.0, 0.0)
            active = False
        self.last_active = active
        if self.motor is not None:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = velocity
            self.motor.publish(cmd)
        status = motor_guard_status(
            armed=self.armed, velocity=velocity, request=request,
            pause_ready=self.pause_client.service_is_ready() and not self.pause_error,
            exclusive=exclusive)
        if self.pause_error or self.emergency_latched:
            status['guard_state'] = 'ABORT'
        status['emergency_latched'] = self.emergency_latched
        if self.nominal_stop_latched:
            status['segment_id'] = self.active_segment_id
        status['nominal_stop_latched'] = self.nominal_stop_latched
        status['segment_rearm_required'] = self.nominal_stop_latched
        status['expected_next_segment_id'] = self.expected_next_segment_id
        status['rearm_prepared_for'] = (
            self.rearm_prepared[1] if self.rearm_prepared else None
        )
        status['rearm_previous_segment_id'] = (
            self.rearm_prepared[0] if self.rearm_prepared else None
        )
        msg = String()
        msg.data = json.dumps(status, allow_nan=False)
        self.guard_output.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=('geofence', 'guard'))
    parser.add_argument('--armed', action='store_true')
    parser.add_argument('--restore-locked-from', type=Path)
    args = parser.parse_args()
    checkpoint = ROOT / 'runtime/pulse1_v6_stop_checkpoint.json'
    restore_id = None

    if args.restore_locked_from is not None:
        if not args.armed:
            raise SystemExit('RESTORE_REQUIRES_ARMED')
        if args.restore_locked_from.resolve() != checkpoint.resolve():
            raise SystemExit('CHECKPOINT_PATH_MISMATCH')
        try:
            snapshot = json.loads(checkpoint.read_text())
            restore_id = locked_segment_from_checkpoint(snapshot)
        except (OSError, ValueError) as exc:
            raise SystemExit(f'RESTORE_DENIED: {exc}')
    elif args.armed and checkpoint.exists():
        raise SystemExit('LOCKED_CHECKPOINT_REQUIRES_RESTORE')
    if args.armed:
        envvar = ('PULSE1_V6_GEOFENCE_ARM' if args.role == 'geofence'
                  else 'PULSE1_V6_MOTOR_ARM')
        if os.environ.get(envvar) != 'YES':
            raise SystemExit(f'Armamento negato: manca {envvar}=YES')
        if not scenario_s1_avoid():
            raise SystemExit('Armamento negato: scenario non S1/AVOID')
    rclpy.init(args=[])
    node = None
    try:
        node = V6Geofence(args.armed) if args.role == 'geofence' else V6MotorGuard(args.armed)
        if restore_id is not None:
            restore_locked_node(node, args.role, restore_id)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
