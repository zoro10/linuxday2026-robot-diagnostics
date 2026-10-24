#!/usr/bin/env python3
"""PULSE-1 V6: only first 10 cm, manual approval and three-arm fail closed.

No whole-mission automation. No ROS actuation in default --plan mode.
DO NOT run --execute until the V5->V6 handover has been independently verified.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

from pulse1_waypoint_plan import load_world, build_route
from pulse1_v6_single_segment_policy import (
    LENGTH_M, first_request, resumed_first_leg_request, stop_request, start_ok, progress_ok, reached, step_count,
    should_emit_nominal_hold,
)
from pulse1_v6_safety_contract import observer_healthy, pose_valid
from pulse1_v6_segment_fsm import V6SegmentGate
from pulse1_v6_route_policy import next_route_request
from pulse1_v6_turn_policy import (
    turn_request, turn_progress_ok,
    turn_reached, turn_step_count,
)

ROOT=Path(__file__).resolve().parent.parent
WORLD=ROOT/'runtime/pulse1_world.json'




def precision_waypoint0_request(world, gz_pose, odom_pose, segment_id, side):
    """One-off WP0 advance proposal. Does not authorize motion."""
    from pulse1_v6_route_policy import valid_request

    if side != "LEFT":
        raise ValueError("PRECISION_SIDE_INVALID")

    for pose in (gz_pose, odom_pose):
        if not (
            isinstance(pose, (tuple, list))
            and len(pose) == 3
            and all(type(v) in (int, float) and math.isfinite(v)
                    for v in pose)
        ):
            raise ValueError("PRECISION_POSE_INVALID")

    gx, gy, gyaw = gz_pose
    ox, oy, oyaw = odom_pose

    from pulse1_waypoint_plan import build_route
    corner_x = float(
        build_route(world, side)['waypoints'][0][0]
    )
    if not math.isfinite(corner_x):
        raise ValueError('PRECISION_CORNER_INVALID')

    if not (
        corner_x - 0.04 <= gx <= corner_x - 0.01
        and abs(gy) <= 0.01
        and abs(oy) <= 0.01
        and abs(gyaw) <= 0.035
        and abs(oyaw) <= 0.035
        and abs(gyaw - oyaw) <= 0.035
        and 0.0 <= ox - gx <= 0.020
    ):
        raise ValueError("PRECISION_POSE_DISAGREEMENT")

    from_gazebo = next_route_request(
        world, gz_pose, segment_id, side
    )
    from_odom = next_route_request(
        world, odom_pose, segment_id, side
    )

    if not (
        from_gazebo["kind"] == "ADVANCE"
        and from_gazebo["waypoint_index"] == 0
        and 0.020 <= from_gazebo["limit"] <= 0.035
        and from_odom["kind"] == "TURN"
        and from_odom["waypoint_index"] == 1
        and from_odom["direction"] == "LEFT"
    ):
        raise ValueError("PRECISION_ROUTE_DISAGREEMENT_UNEXPECTED")

    request = dict(from_gazebo)

    # Il movimento resta riferito all'odometria ROS.
    request["start_pose"] = [float(v) for v in odom_pose]

    if not valid_request(request):
        raise ValueError("PRECISION_REQUEST_INVALID")

    return request


def segment_progress_ok(pose, origin, request):
    if request["kind"] == "TURN":
        return turn_progress_ok(
            pose, origin, request["direction"], request["limit"]
        )
    return progress_ok(pose, origin, request['limit'])


def segment_reached(pose, origin, request):
    if request["kind"] == "TURN":
        return turn_reached(
            pose, origin, request["direction"], request["limit"]
        )
    return reached(pose, origin, request['limit'])


def segment_steps(pose, origin, dt, request):
    if request["kind"] == "TURN":
        return turn_step_count(
            pose, origin, request["direction"],
            request["limit"], dt
        )
    return step_count(pose, origin, dt, request['limit'])


def segment_should_emit_nominal_hold(watchdog, request, pose):
    if not isinstance(request, dict):
        return False

    if (request.get("kind") == "ADVANCE"
            and request.get("limit") == LENGTH_M):
        return should_emit_nominal_hold(watchdog, request, pose)

    return (
        isinstance(watchdog, dict)
        and request.get("phase") == "DRIVE"
        and watchdog.get("segment_id") == request.get("segment_id")
        and watchdog.get("action") == "STOPPING"
        and watchdog.get("reason") == "TARGET_REACHED"
        and segment_reached(pose, request["start_pose"], request)
    )


def verify_next_segment_committed(guard, geofence, next_id):
    if not (
        isinstance(next_id, str)
        and 8 <= len(next_id) <= 64
        and guard.get("expected_next_segment_id") == next_id
        and geofence.get("rearm_next_segment_id") == next_id
        and geofence.get("action") == "WAIT"
        and geofence.get("reason") == "REARM_VERIFIED"
        and guard.get("emergency_latched") is False
        and guard.get("nominal_stop_latched") is False
    ):
        raise RuntimeError("NEXT_SEGMENT_NOT_COMMITTED_BY_BOTH_SAFETIES")



def warmup_overrun_permitted(
    *, warmup, extra, paused, ack, request_absent,
    guard, geofence, supervisor, cmd,
    guard_age, gf_age, supervisor_age, cmd_age
):
    """Bounded exception ONLY for idle warm-up; never for motion."""
    if not (
        warmup is True
        and type(extra) is int
        and 1 <= extra <= 50
        and paused is True
        and ack is True
        and request_absent is True
        and isinstance(guard, dict)
        and isinstance(geofence, dict)
        and isinstance(supervisor, dict)
        and isinstance(cmd, (tuple, list))
        and len(cmd) == 2
        and all(
            type(v) in (int, float)
            and math.isfinite(v)
            and abs(v) < 1e-8
            for v in cmd
        )
        and guard.get('guard_state') == 'BLOCKED'
        and guard.get('nominal_stop_latched') is False
        and guard.get('emergency_latched') is False
        and geofence.get('action') == 'WAIT'
        and supervisor.get('state') == 'HEALTHY'
    ):
        return False

    expected = guard.get('expected_next_segment_id')
    reason = geofence.get('reason')

    nominal_rearm = (
        reason == 'REARM_VERIFIED'
        and isinstance(expected, str)
        and 8 <= len(expected) <= 64
        and geofence.get('rearm_next_segment_id') == expected
    )

    authorized_cold_start = (
        reason == 'NOT_STARTED'
        and os.environ.get('PULSE1_V6_COLD_RECOVERY') == 'YES'
        and os.environ.get(
            'PULSE1_V6_COLD_RECOVERY_OPERATOR'
        ) == 'YES'
        and expected in (None, '')
        and geofence.get('rearm_next_segment_id') in (None, '')
        and guard.get('segment_id') in (None, '')
        and geofence.get('segment_id') in (None, '')
    )

    if not (nominal_rearm or authorized_cold_start):
        return False

    return all(
        type(age) in (int, float)
        and math.isfinite(age)
        and 0 <= age < limit
        for age, limit in (
            (guard_age, 0.6),
            (gf_age, 0.6),
            (supervisor_age, 0.8),
            (cmd_age, 0.6),
        )
    )


def plan():
    w=load_world(WORLD)
    r=build_route(w,'LEFT')
    d=first_request(w,(0.,0.,0.))
    print(json.dumps({'operation':'FIRST_V6_SEGMENT_ONLY','start':[0,0,0],
                      'limit_m':LENGTH_M,'target_waypoint':d['target'],
                      'total_waypoints':len(r['waypoints']),
                      'motion_authorized':False,'requires_manual_handover':True,
                      'separate_arm':'PULSE1_V6_EXECUTE=YES and --execute'},indent=2))


def check_graph():
    """Read-only ROS graph and world-pause audit; creates no publishers."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import WorldStatistics
    rclpy.init(args=[])
    node=None
    try:
        node=Node('pulse1_v6_readonly_audit')
        state={'paused':None}
        node.create_subscription(WorldStatistics,'/world/empty/stats',
                                 lambda m:state.update(paused=bool(m.paused)),
                                 qos_profile_sensor_data)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if state['paused'] is not None:break
        names={n for n,ns in node.get_node_names_and_namespaces() if ns=='/'}
        pubs=node.get_publishers_info_by_topic('/model/pulse1_proto/cmd_vel')
        old={name for name in ('pulse1_motor_guard_live','pulse1_gazebo_supervisor',
               'pulse1_s1_geofence_watchdog_v2') if name in names}
        report={'world_paused':state['paused'],
          'cmd_vel_publishers':[p.node_name for p in pubs],
          'legacy_v5_nodes_present':sorted(old),
          'v6_geofence_node':'pulse1_v6_geofence_watchdog' in names,
          'v6_guard_node':'pulse1_v6_motor_guard' in names,
          'v6_supervisor_node':'pulse1_v6_supervisor' in names,
          'request_publishers':len(node.get_publishers_info_by_topic('/pulse1/v6_segment_request')),
          'world_control_services':[n for n,t in node.get_service_names_and_types()
             if n=='/world/empty/control'],
          'motion_authorized':False}
        print(json.dumps(report,indent=2,ensure_ascii=False))
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def execute():
    if os.environ.get('PULSE1_V6_EXECUTE') != 'YES':
        raise RuntimeError('NO_EXPLICIT_V6_EXECUTE_CONSENT')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from ros_gz_interfaces.msg import WorldStatistics
    from ros_gz_interfaces.srv import ControlWorld
    from pulse1_v6_ros_safety import pose_from_odom, CMD_TOPIC

    class Runner(Node):
        def __init__(self):
            super().__init__('pulse1_v6_single_segment_executor')
            self.warmup_resync_used = False
            self.val={};self.seen={};self.req=None;self.last_pub=0
            for key, topic in (
                ('observer','/pulse1/avoid_observer'),
                ('guard','/pulse1/v6_motor_guard'),
                ('watchdog','/pulse1/v6_geofence_watchdog'),
                ('supervisor','/pulse1/v6_supervisor_status')):
                self.create_subscription(String,topic,
                    lambda m,k=key:self.on_json(k,m),10)
            self.create_subscription(Odometry,'/model/pulse1_proto/odometry',self.on_pose,qos_profile_sensor_data)
            self.create_subscription(Twist,CMD_TOPIC,self.on_cmd,10)
            self.create_subscription(WorldStatistics,'/world/empty/stats',self.on_stats,qos_profile_sensor_data)
            self.pub=self.create_publisher(String,'/pulse1/v6_segment_request',10)
            self.client=self.create_client(ControlWorld,'/world/empty/control')
            self.create_timer(.05,self.heartbeat)

        def on_json(self,k,m):
            try: d=json.loads(m.data)
            except (ValueError,TypeError): d=None
            self.val[k]=d if isinstance(d,dict) else None;self.seen[k]=time.monotonic()
            # STOPPING->HOLD MUST be immediate. Otherwise the 50 ms Geofence
            # timer may reject the still-DRIVE request before ControlWorld ACK.
            if k=='watchdog' and segment_should_emit_nominal_hold(
                    self.val[k],self.req,self.val.get('pose')):
                self.req=stop_request(self.req)
                self.heartbeat()
        def on_pose(self,m):
            self.val['pose']=pose_from_odom(m);self.seen['pose']=time.monotonic()
            if segment_should_emit_nominal_hold(self.val.get('watchdog'),self.req,self.val['pose']):
                self.req=stop_request(self.req)
                self.heartbeat()
        def on_cmd(self,m):
            self.val['cmd']=(m.linear.x,m.angular.z);self.seen['cmd']=time.monotonic()
        def on_stats(self,m):
            sim=m.sim_time.sec + m.sim_time.nanosec*1e-9
            self.val['stats']=(int(m.iterations),bool(m.paused),float(sim))
            self.seen['stats']=time.monotonic()
        def age(self,k):
            return time.monotonic()-self.seen[k] if k in self.seen else None
        def heartbeat(self):
            if self.req is not None:
                m=String();m.data=json.dumps(self.req,allow_nan=False)
                self.pub.publish(m);self.last_pub=time.monotonic()
        def poll(self,seconds=.02):
            rclpy.spin_once(self,timeout_sec=seconds)
            gf=self.val.get('watchdog')
            guard=self.val.get('guard')
            supervisor=self.val.get('supervisor')
            if isinstance(gf,dict) and gf.get('action')=='PAUSE':
                raise RuntimeError('GEOFENCE_ABORT: '+str(gf.get('reason')))
            if isinstance(guard,dict) and (guard.get('emergency_latched') is True or guard.get('guard_state')=='ABORT'):
                raise RuntimeError('GUARD_ABORT')
            if isinstance(supervisor,dict) and supervisor.get('state')=='EMERGENCY_PAUSE':
                raise RuntimeError('SUPERVISOR_EMERGENCY')
        def until(self,test,secs,label):
            deadline=time.monotonic()+secs
            while time.monotonic()<deadline:
                self.poll()
                if test():return
            raise RuntimeError('TIMEOUT_'+label)
        def stats(self):
            s=self.val.get('stats')
            if s is None or self.age('stats') is None or self.age('stats')>.8:
                raise RuntimeError('STATS_STALE')
            return s
        def verify_quiescent_world(self):
            # Nessuno step finché il contatore non è stabile
            # in almeno due nuovi messaggi WorldStatistics.
            initial = self.stats()
            if initial[1] is not True:
                raise RuntimeError('WARMUP_WORLD_NOT_PAUSED')

            baseline = initial[0]
            last_stamp = self.seen.get('stats')
            fresh_samples = 0
            deadline = time.monotonic() + 3.0

            while time.monotonic() < deadline:
                self.poll()
                current = self.stats()

                if current[1] is not True:
                    raise RuntimeError('WARMUP_PAUSE_LOST')

                if current[0] != baseline:
                    raise RuntimeError(
                        f'WARMUP_WORLD_NOT_STABLE: '
                        f'before={baseline} observed={current[0]}'
                    )

                stamp = self.seen.get('stats')
                if stamp != last_stamp:
                    fresh_samples += 1
                    last_stamp = stamp

                    if fresh_samples >= 2:
                        return

            raise RuntimeError('WARMUP_STABLE_STATS_TIMEOUT')

        def physics(self,n,timeout=25.,terminal=False,warmup=False):
            if type(n) is not int or not 1 <= n <= 1000:
                raise RuntimeError('INVALID_PHYSICS_BUDGET')

            if warmup:
                if self.req is not None:
                    raise RuntimeError('WARMUP_AFTER_REQUEST_DENIED')
                self.verify_quiescent_world()

            s=self.stats()
            if s[1] is not True or not self.client.service_is_ready():
                raise RuntimeError('NOT_PAUSED_OR_CONTROL_DOWN')
            before=s[0]; target=before+n
            req=ControlWorld.Request();req.world_control.pause=True;req.world_control.multi_step=n
            future=self.client.call_async(req)
            deadline=time.monotonic()+timeout
            ack=False
            while time.monotonic()<deadline:
                self.poll()
                s2=self.val.get('stats')
                if s2 is not None and s2[0]>target and (
                        not warmup or s2[0]-target>50):
                    raise RuntimeError(
                        f'PHYSICS_OVERRUN: '
                        f'before={before} requested={n} '
                        f'target={target} observed={s2[0]} '
                        f'extra={s2[0]-target} '
                        f'paused={s2[1]} ack={ack}'
                    )
                if future.done() and not ack:
                    response=future.result()
                    if response is None or response.success is not True:
                        raise RuntimeError('CONTROL_ACK_DENIED')
                    ack=True
                if ack and s2 and s2[1] is True and s2[0]>=target:
                    extra = s2[0] - target
                    if extra:
                        allowed = (
                            not self.warmup_resync_used
                            and warmup_overrun_permitted(
                            warmup=warmup,
                            extra=extra,
                            paused=s2[1],
                            ack=ack,
                            request_absent=(self.req is None),
                            guard=self.val.get('guard'),
                            geofence=self.val.get('watchdog'),
                            supervisor=self.val.get('supervisor'),
                            cmd=self.val.get('cmd'),
                            guard_age=self.age('guard'),
                            gf_age=self.age('watchdog'),
                            supervisor_age=self.age('supervisor'),
                            cmd_age=self.age('cmd'),
                            )
                        )
                        if not allowed:
                            raise RuntimeError(
                                f'WARMUP_OVERRUN_UNSAFE: extra={extra}'
                            )
                        print(
                            f'WARN WARMUP_EXTRA_TICKS: '
                            f'extra={extra} requested={n} '
                            f'actual={s2[0]-before} '
                            f'PAUSED/ACK/BLOCKED/ZERO verified',
                            flush=True
                        )

                    if warmup and extra > 0:
                        self.warmup_resync_used = True

                    return (
                        (s2[2]-s[2]) / (s2[0]-before),
                        'NORMAL'
                    )
                if terminal and ack and s2 and s2[1] is True and before<s2[0]<target:
                    gf=self.val.get('watchdog') or {}
                    if gf.get('action') in ('STOPPING','COMPLETE') and gf.get('reason')=='TARGET_REACHED':
                        return (s2[2]-s[2])/(s2[0]-before),'TERMINAL'
            # Never repeat a pending ControlWorld step.
            raise RuntimeError('CONTROL_PENDING_OR_TIMEOUT')

        def nodes_present(self):
            names={name for name,namespace in self.get_node_names_and_namespaces() if namespace=='/'}
            old={'pulse1_motor_guard_live','pulse1_gazebo_supervisor','pulse1_s1_geofence_watchdog_v2'}
            return bool(names & old)
        def preflight(self):
            self.until(lambda:self.val.get('stats') is not None
                       and isinstance(self.val.get('guard'),dict)
                       and isinstance(self.val.get('watchdog'),dict)
                       and isinstance(self.val.get('supervisor'),dict)
                       and self.val.get('cmd') is not None,8.,'INPUT_DISCOVERY')
            self.until(lambda:self.client.service_is_ready(),6.,'CONTROL_DISCOVERY')
            def check():
                s=self.stats()
                return start_ok(world_paused=s[1],world_age=self.age('stats'),
                    cmd=self.val.get('cmd'),cmd_age=self.age('cmd'),
                    guard=self.val.get('guard'),guard_age=self.age('guard'),
                    watchdog=self.val.get('watchdog'),watchdog_age=self.age('watchdog'),
                    v5_nodes=self.nodes_present(),
                    publisher_count=(1 if len(self.get_publishers_info_by_topic(CMD_TOPIC))==1
                       and self.get_publishers_info_by_topic(CMD_TOPIC)[0].node_name=='pulse1_v6_motor_guard' else -1),
                    request_publishers=(1 if len(self.get_publishers_info_by_topic('/pulse1/v6_segment_request'))==1
                       and len(self.get_subscriptions_info_by_topic('/pulse1/v6_segment_request'))==2
                       and {s.node_name for s in self.get_subscriptions_info_by_topic('/pulse1/v6_segment_request')}==
                       {'pulse1_v6_motor_guard','pulse1_v6_geofence_watchdog'} else -1),
                    service_ready=self.client.service_is_ready(),
                    supervisor=self.val.get('supervisor'),supervisor_age=self.age('supervisor'))
            # ROS discovery is asynchronous. All original start_ok
            # conditions must remain true for one continuous second.
            # Never relax an individual safety predicate.
            deadline = time.monotonic() + 8.0
            stable_since = None
            while time.monotonic() < deadline:
                self.poll()
                if check():
                    if stable_since is None:
                        stable_since = time.monotonic()
                    if time.monotonic() - stable_since >= 1.0:
                        print('PREFLIGHT STABLE: 1.0s', flush=True)
                        return check
                else:
                    stable_since = None
            raise RuntimeError('PREFLIGHT_DENIED: V6 armed/exclusive, V5 absent, paused, zero and supervisor required')

    rclpy.init(args=[])
    runner=None; status='UNSTARTED'; phase='BOOT'; old_req=None
    try:
        runner=Runner()
        check=runner.preflight()
        print('PRECHECK OK: V6 exclusive / old guard absent / pause / zero / V6 supervisor',flush=True)
        dt=None
        for count in (250,250,500,1000):
            if not check(): raise RuntimeError('WARMUP_PREFLIGHT_CHANGED')
            dt,_=runner.physics(count,warmup=True)
            print(f'WARMUP {count} physics ticks, dt={dt:.6f}',flush=True)
            obs=runner.val.get('observer')
            if (observer_healthy(obs,runner.age('observer'))
                and pose_valid(runner.val.get('pose'))
                and runner.age('pose') is not None and runner.age('pose')<.5):
                break
        else:
            raise RuntimeError('WARMUP_SENSORS_NOT_READY')
        if not check():raise RuntimeError('AFTER_WARMUP_PREFLIGHT_CHANGED')
        origin=runner.val['pose']

        # TASK 172: UNCONDITIONAL DIAGNOSTIC EXIT.
        # No segment request can be constructed after this point.
        import json as _diagnostic_json

        raw_gz = os.environ['PULSE1_V6_COLD_GZ_POSE']
        gz_pose = tuple(float(v) for v in raw_gz.split(','))
        ros_pose = tuple(float(v) for v in origin)

        if len(gz_pose) != 3 or len(ros_pose) != 3:
            raise RuntimeError('INVALID_DIAGNOSTIC_POSE')

        delta = tuple(
            a - b for a, b in zip(ros_pose, gz_pose)
        )

        report = {
            'ros_odom_pose': ros_pose,
            'gazebo_snapshot': gz_pose,
            'delta_ros_minus_gazebo': delta,
            'ros_pose_age_s': runner.age('pose'),
            'guard': (runner.val.get('guard') or {}).get(
                'guard_state'),
            'geofence': (runner.val.get('watchdog') or {}).get(
                'action'),
            'supervisor': (runner.val.get('supervisor') or {}).get(
                'state'),
            'warmup_resync_used': runner.warmup_resync_used,
            'difference_exceeds_1cm': any(
                abs(v) > .01 for v in delta
            ),
        }

        print('=== TASK 172 POSE DIAGNOSTIC ===', flush=True)
        print(
            _diagnostic_json.dumps(
                report, indent=2, allow_nan=False
            ),
            flush=True
        )
        print(
            'DIAGNOSTIC EXIT: ZERO SEGMENT REQUESTS',
            flush=True
        )
        return
        if os.environ.get('PULSE1_V6_ROUTE_NEXT') == 'YES':
            if (
                os.environ.get('PULSE1_V6_TURN') == 'YES'
                or os.environ.get('PULSE1_V6_RESUME_FIRST_LEG') == 'YES'
            ):
                raise RuntimeError('ROUTE_MODES_CONFLICT')

            next_id = os.environ.get('PULSE1_V6_SEGMENT_ID', '')
            current_guard = runner.val.get('guard') or {}
            current_geofence = runner.val.get('watchdog') or {}

            verify_next_segment_committed(
                current_guard, current_geofence, next_id
            )

            side = os.environ.get('PULSE1_V6_ROUTE_SIDE', 'LEFT')
            if os.environ.get("PULSE1_V6_PRECISION_WP0") == "YES":
                raw = os.environ.get(
                    "PULSE1_V6_PRECISION_GZ_POSE", ""
                )
                try:
                    gz_snapshot = tuple(
                        float(v) for v in raw.split(",")
                    )
                    snapshot_time = float(os.environ.get(
                        "PULSE1_V6_PRECISION_GZ_TIME", "nan"
                    ))
                except ValueError:
                    raise RuntimeError("PRECISION_SNAPSHOT_INVALID")

                snapshot_age = time.monotonic() - snapshot_time

                if not (
                    math.isfinite(snapshot_age)
                    and 0 <= snapshot_age <= 60
                ):
                    raise RuntimeError("PRECISION_SNAPSHOT_STALE")

                if not check():
                    raise RuntimeError("PRECISION_SAFETY_CHANGED")

                request = precision_waypoint0_request(
                    load_world(WORLD), gz_snapshot,
                    origin, next_id, side
                )

                print(
                    "PRECISION WP0: "
                    f"ADVANCE limit={request['limit']:.6f}",
                    flush=True
                )
            else:
                request = next_route_request(
                    load_world(WORLD), origin, next_id, side
                )

            # BEGIN_ROUTE_PLAN_LOCK
            expected_kind = os.environ.get("PULSE1_V6_EXPECT_KIND")
            expected_wp = os.environ.get("PULSE1_V6_EXPECT_WP")
            expected_limit = os.environ.get("PULSE1_V6_EXPECT_LIMIT")

            if any(v is not None for v in (
                expected_kind, expected_wp, expected_limit
            )):
                if (expected_kind not in ("ADVANCE", "TURN")
                        or expected_wp is None
                        or expected_limit is None):
                    raise RuntimeError("PLAN_LOCK_INCOMPLETE")

                try:
                    wp = int(expected_wp)
                    ceiling = float(expected_limit)
                except (ValueError, TypeError):
                    raise RuntimeError("PLAN_LOCK_INVALID")

                if not math.isfinite(ceiling) or ceiling <= 0:
                    raise RuntimeError("PLAN_LOCK_INVALID")

                if (request.get("kind") != expected_kind
                        or request.get("waypoint_index") != wp):
                    raise RuntimeError("PLAN_EXECUTOR_DIVERGED")

                actual_limit = request.get("limit")
                if (type(actual_limit) not in (int, float)
                        or not math.isfinite(actual_limit)
                        or actual_limit <= 0
                        or actual_limit > ceiling + 1e-6):
                    raise RuntimeError("PLAN_LIMIT_EXCEEDED")
            # END_ROUTE_PLAN_LOCK

        elif os.environ.get('PULSE1_V6_TURN') == 'YES':
            if os.environ.get('PULSE1_V6_RESUME_FIRST_LEG') == 'YES':
                raise RuntimeError('TURN_ADVANCE_MODES_CONFLICT')

            next_id = os.environ.get('PULSE1_V6_SEGMENT_ID', '')
            current_guard = runner.val.get('guard') or {}
            current_geofence = runner.val.get('watchdog') or {}

            verify_next_segment_committed(
                current_guard, current_geofence, next_id
            )

            direction = os.environ.get(
                'PULSE1_V6_TURN_DIRECTION', 'LEFT'
            )
            request = turn_request(
                load_world(WORLD), origin, next_id, direction
            )


        elif os.environ.get('PULSE1_V6_COLD_RECOVERY') == 'YES':
            import secrets

            if os.environ.get(
                'PULSE1_V6_COLD_RECOVERY_OPERATOR'
            ) != 'YES':
                raise RuntimeError('COLD_RECOVERY_CONSENT_MISSING')

            if any(
                os.environ.get(k) == 'YES'
                for k in (
                    'PULSE1_V6_ROUTE_NEXT',
                    'PULSE1_V6_TURN',
                    'PULSE1_V6_RESUME_FIRST_LEG',
                )
            ):
                raise RuntimeError('COLD_RECOVERY_MODE_CONFLICT')

            g = runner.val.get('guard') or {}
            f = runner.val.get('watchdog') or {}
            sup = runner.val.get('supervisor') or {}

            cold_safe = all((
                check(),
                g.get('guard_state') == 'BLOCKED',
                g.get('emergency_latched') is False,
                g.get('nominal_stop_latched') is False,
                g.get('expected_next_segment_id') in (None, ''),
                f.get('action') == 'WAIT',
                f.get('reason') == 'NOT_STARTED',
                f.get('ready_for_motion') is False,
                sup.get('state') == 'HEALTHY',
                runner.stats()[1] is True,
                runner.age('guard') is not None,
                runner.age('guard') < .6,
                runner.age('watchdog') is not None,
                runner.age('watchdog') < .6,
                runner.age('supervisor') is not None,
                runner.age('supervisor') < .8,
                runner.age('pose') is not None,
                runner.age('pose') < .5,
            ))

            if not cold_safe:
                raise RuntimeError('COLD_RECOVERY_SAFETY_DENIED')

            try:
                gz_pose = tuple(float(x) for x in
                    os.environ['PULSE1_V6_COLD_GZ_POSE'].split(','))
            except (KeyError, ValueError):
                raise RuntimeError('COLD_GAZEBO_POSE_MISSING')

            if (
                len(gz_pose) != 3
                or not all(math.isfinite(v) for v in gz_pose)
                or any(abs(a-b) > .01
                       for a, b in zip(origin, gz_pose))
                or not 1.92 <= origin[0] <= 1.97
                or abs(origin[1]) > .01
                or abs(origin[2]) > .03
            ):
                raise RuntimeError('COLD_POSE_MISMATCH')

            next_id = 'v6-cold-' + secrets.token_hex(10)

            request = resumed_first_leg_request(
                load_world(WORLD), origin, next_id
            )

            distance = request['target'][0] - origin[0]

            if not (
                request['kind'] == 'ADVANCE'
                and request['waypoint_index'] == 0
                and abs(request['limit'] - .1) < 1e-6
                and .14 < distance < .17
                and request['limit'] < distance - .03
            ):
                raise RuntimeError('COLD_SEGMENT_GEOMETRY_DENIED')

            print(
                f'COLD RECOVERY AUTHORIZED: {next_id} '
                f'origin={origin} limit={request["limit"]:.6f}',
                flush=True
            )

        elif os.environ.get('PULSE1_V6_RESUME_FIRST_LEG') == 'YES':
            next_id = os.environ.get('PULSE1_V6_SEGMENT_ID', '')
            current_guard = runner.val.get('guard') or {}
            current_geofence = runner.val.get('watchdog') or {}

            if not (
                len(next_id) >= 8
                and current_guard.get('expected_next_segment_id') == next_id
                and current_geofence.get('rearm_next_segment_id') == next_id
                and current_geofence.get('action') == 'WAIT'
                and current_geofence.get('reason') == 'REARM_VERIFIED'
            ):
                raise RuntimeError('RESUME_NOT_COMMITTED_BY_BOTH_SAFETIES')

            request=resumed_first_leg_request(
                load_world(WORLD), origin, next_id
            )
        else:
            request=first_request(load_world(WORLD),origin)

        gate=V6SegmentGate()
        pre_stats=runner.stats()
        gate_phase=gate.prepare_executor_request(
            request, origin,
            paused=(pre_stats[1] is True),
            paused_age_s=runner.age('stats'),
            guard=runner.val.get('guard'),
            guard_age_s=runner.age('guard'),
            cmd=runner.val.get('cmd'),
            cmd_age_s=runner.age('cmd'),
        )
        if gate_phase != 'PREPARED':
            raise RuntimeError('FSM_PREPARE_FAILED:'+str(gate.abort_reason))

        runner.req=request
        phase='DRIVE'
        runner.until(lambda: (runner.val.get('watchdog') or {}).get('ready_for_motion') is True
           and (runner.val.get('guard') or {}).get('guard_state')=='AUTHORIZED',1.,'V6_DUAL_AUTH')
        g_auth=runner.val.get('guard') or {}
        w_auth=runner.val.get('watchdog') or {}
        gate_phase=gate.record_authorized_motion(
            now_s=time.monotonic(),
            v6_watchdog_confirmed=(
                w_auth.get('source') == 'v6_geofence_watchdog'
                and w_auth.get('segment_id') == request['segment_id']
                and w_auth.get('armed') is True
                and w_auth.get('ready_for_motion') is True
            ),
            v6_motor_authorized=(
                g_auth.get('source') == 'motor_guard_live'
                and g_auth.get('segment_id') == request['segment_id']
                and g_auth.get('guard_state') == 'AUTHORIZED'
                and g_auth.get('v6_armed') is True
                and g_auth.get('motor_commands') is True
                and g_auth.get('cmd_vel_exclusive') is True
                and g_auth.get('emergency_latched') is False
                and g_auth.get('nominal_stop_latched') is False
            ),
        )
        if gate_phase != 'ACTIVE':
            raise RuntimeError('FSM_AUTHORIZATION_FAILED:'+str(gate.abort_reason))

        print(
            f"ROUTE SEGMENT: {request['kind']} "
            f"waypoint={request['waypoint_index']} "
            f"limit={request['limit']:.6f}",
            flush=True
        )
        print(f'DUAL AUTH OK: {request["kind"]}, limit={request["limit"]:.6f}',flush=True)
        beg=time.monotonic();steps=0;last_world_step_ack=False
        while True:
            if time.monotonic()-beg>22:raise RuntimeError('SEGMENT_EXEC_TIMEOUT')
            pose=runner.val.get('pose')
            gate_phase=gate.progress(
                pose, now_s=time.monotonic(),
                pose_age_s=runner.age('pose'),
                reached_tolerance_m=.012,
            )
            if gate_phase == 'ABORT':
                raise RuntimeError('FSM_PROGRESS_ABORT:'+str(gate.abort_reason))
            if not segment_progress_ok(pose,origin,request):raise RuntimeError('SEGMENT_ENVELOPE_BREACHED')
            gf=runner.val.get('watchdog') or {}
            if gf.get('action')=='STOPPING' and gf.get('reason')=='TARGET_REACHED':
                if not segment_reached(pose,origin,request):raise RuntimeError('PREMATURE_STOPPING')
                break
            if segment_reached(pose,origin,request):
                runner.until(lambda:(runner.val.get('watchdog') or {}).get('action')=='STOPPING',.35,'STOPPING_EXPECTED')
                break
            if gf.get('action')!='MONITOR' or gf.get('ready_for_motion') is not True:
                raise RuntimeError('LOST_GEOFENCE_LEASE')
            if not (runner.val.get('guard') or {}).get('guard_state')=='AUTHORIZED':
                raise RuntimeError('GUARD_NOT_AUTHORIZED')
            n=segment_steps(pose,origin,dt,request)
            dt,st=runner.physics(n,timeout=5.,terminal=True)
            last_world_step_ack=True
            steps+=1
            if st=='TERMINAL':
                if gate.phase == 'ACTIVE':
                    gate_phase=gate.progress(
                        runner.val.get('pose'), now_s=time.monotonic(),
                        pose_age_s=runner.age('pose'),
                        reached_tolerance_m=.012,
                    )
                    if gate_phase == 'ABORT':
                        raise RuntimeError('FSM_TERMINAL_PROGRESS_ABORT:'+str(gate.abort_reason))
                elif gate.phase != 'STOP_REQUESTED':
                    raise RuntimeError('FSM_TERMINAL_PHASE_INVALID:'+str(gate.phase))
                print('CONTROL_STEP interrupted by nominal PAUSE',flush=True)
                break
            if steps%5==0:print(f'block {steps}, x={runner.val["pose"][0]:.4f}',flush=True)
        if gate.phase != 'STOP_REQUESTED':
            raise RuntimeError('FSM_STOP_NOT_REQUESTED:'+str(gate.phase))
        if not segment_reached(runner.val.get('pose'),origin,request):
            raise RuntimeError('STOP_POSITION_NOT_REACHED')
        runner.req=stop_request(request)
        phase='STOPPING'
        runner.until(lambda:(runner.val.get('watchdog') or {}).get('action')=='COMPLETE',3.,'INDEPENDENT_STOP_ACK')
        g=runner.val.get('guard') or {}; w=runner.val.get('watchdog') or {}
        s=runner.stats(); c=runner.val.get('cmd')
        if not (w.get('reason')=='TARGET_REACHED' and w.get('pause_ack') is True
                and g.get('nominal_stop_latched') is True
                and g.get('segment_id')==request['segment_id']
                and g.get('emergency_latched') is False
                and s[1] is True
                and isinstance(c,(list,tuple)) and len(c)==2
                and all(abs(v)<1e-8 for v in c)):
            raise RuntimeError('STOP_EVIDENCE_INCOMPLETE')
        gate_phase=gate.acknowledge_stop(
            runner.val.get('pose'),
            now_s=time.monotonic(),
            pose_age_s=runner.age('pose'),
            world_step_ack=(last_world_step_ack is True),
            paused=(s[1] is True),
            paused_age_s=runner.age('stats'),
            guard=g,
            guard_age_s=runner.age('guard'),
            cmd=c,
            cmd_age_s=runner.age('cmd'),
            independent_stop_ack=(
                w.get('source') == 'v6_geofence_watchdog'
                and w.get('action') == 'COMPLETE'
                and w.get('reason') == 'TARGET_REACHED'
                and w.get('pause_ack') is True
                and w.get('segment_id') == request['segment_id']
            ),
        )
        if gate_phase != 'STOP_CONFIRMED':
            raise RuntimeError('FSM_STOP_ACK_REJECTED:'+str(gate.abort_reason))

        status='SUCCESS'
        print(f'SUCCESS V6: pose={runner.val["pose"]} GF COMPLETE, GUARD NOMINAL STOP, PAUSED',flush=True)
    except Exception as exc:
        status='ABORT';print('ABORT V6:',exc,flush=True)
        if runner is not None:
            try:
                if runner.req is not None:
                    runner.req={**runner.req,'phase':'HOLD','reason':'ABORT',
                               'proposed_linear_m_s':0.0,'proposed_angular_rad_s':0.0}
                # Explicit PAUSE on any abort; never retry a ControlWorld step.
                if runner.client.service_is_ready():
                    q=ControlWorld.Request();q.world_control.pause=True
                    q.world_control.multi_step=0
                    f=runner.client.call_async(q)
                    deadline=time.monotonic()+2
                    while time.monotonic()<deadline and not f.done():
                        rclpy.spin_once(runner,timeout_sec=.02)
            except Exception: pass
        raise
    finally:
        if runner is not None:
            runner.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser()
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--plan',action='store_true')
    modes.add_argument('--execute',action='store_true')
    modes.add_argument('--check',action='store_true')
    args=parser.parse_args()
    if args.check:
        check_graph();return
    if not args.execute:
        plan();return
    execute()

if __name__=='__main__':main()
