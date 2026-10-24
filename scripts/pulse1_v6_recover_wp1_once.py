#!/usr/bin/env python3
"""One-shot recovery of the SPECIFIC already-committed WP1 segment.

NEVER rearms or creates a new segment ID. Deliberately not reusable for a
new incident. Requires an operator opt-in AND both live ROS safety statuses.
"""
import ast
import math
import os
import re
import subprocess
import sys
import time

import rclpy
from pulse1_waypoint_plan import load_world
from pulse1_v6_route_policy import next_route_request, wrap
from pulse1_v6_sequencer import ROOT, WORLD, Monitor, gazebo_pose, v6_corner_planning_pose

PENDING_ID = 'v6-route-34038087d012595319c9'
EVIDENCE = ROOT / 'runtime/evidence/v6_video_test_20261023T120812Z/plan_9.log'


def main():
    if (sys.argv[1:] != ['--execute']
            or os.environ.get('PULSE1_V6_RECOVER_WP1') != 'YES'):
        raise RuntimeError('EXPLICIT_WP1_RECOVERY_CONSENT_REQUIRED')
    text = EVIDENCE.read_text()
    m = re.search(r'CORNER FRAME COMPENSATED: (\([^\n]+\))', text)
    n = re.search(r'NEXT: (ADVANCE|TURN) WAYPOINT: (\d+) LIMIT: ([\d.]+)', text)
    old = re.search(r'VERIFIED STOP: ([\w-]+)', text)
    if not (m and n and old):
        raise RuntimeError('ORIGINAL_PLANNING_EVIDENCE_MISSING')
    recorded = ast.literal_eval(m.group(1))
    kind, wp, ceiling = n.group(1), int(n.group(2)), float(n.group(3))
    if not (kind == 'ADVANCE' and wp == 1 and .02 < ceiling < .04
            and len(recorded) == 3):
        raise RuntimeError('WRONG_PENDING_ROUTE_PLAN')

    def committed(mon):
        g, f = mon.value('guard'), mon.value('gf')
        return (mon.base_safe()
                and g.get('expected_next_segment_id') == PENDING_ID
                and g.get('guard_state') == 'BLOCKED'
                and g.get('nominal_stop_latched') is False
                and g.get('emergency_latched') is False
                and f.get('rearm_previous_segment_id') == old.group(1)
                and f.get('rearm_next_segment_id') == PENDING_ID
                and f.get('action') == 'WAIT'
                and f.get('reason') == 'REARM_VERIFIED')

    rclpy.init(args=[])
    node = Monitor()
    try:
        node.wait(lambda: committed(node), 'WP1_ALREADY_COMMITTED', timeout=8)
        pose = gazebo_pose()
        node.wait(lambda: committed(node), 'WP1_STILL_COMMITTED', timeout=8)
        world = load_world(WORLD)
        virtual = v6_corner_planning_pose(pose, world, 'LEFT')
        if (math.hypot(virtual[0] - recorded[0], virtual[1] - recorded[1]) > .010
                or abs(wrap(virtual[2] - recorded[2])) > .025):
            raise RuntimeError('WP1_POSE_CHANGED_SINCE_REARM')
        physical_plan = next_route_request(world, virtual, PENDING_ID, 'LEFT')
        if (physical_plan['kind'] != kind
                or physical_plan['waypoint_index'] != wp
                or physical_plan['limit'] > ceiling + 1e-6):
            raise RuntimeError('WP1_SIGNED_PLAN_CHANGED')
        stamp = time.monotonic()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    env = dict(os.environ)
    for key in ('PULSE1_V6_PRECISION_WP0', 'PULSE1_V6_PRECISION_GZ_POSE',
                'PULSE1_V6_PRECISION_GZ_TIME', 'PULSE1_V6_TURN',
                'PULSE1_V6_RESUME_FIRST_LEG'):
        env.pop(key, None)
    env.update({
        'PULSE1_V6_EXECUTE': 'YES',
        'PULSE1_V6_ROUTE_NEXT': 'YES',
        'PULSE1_V6_SEGMENT_ID': PENDING_ID,
        'PULSE1_V6_EXPECT_KIND': kind,
        'PULSE1_V6_EXPECT_WP': str(wp),
        'PULSE1_V6_EXPECT_LIMIT': repr(ceiling),
        'PULSE1_V6_ROUTE_SIDE': 'LEFT',
        'PULSE1_V6_ROUTE_GZ_POSE': ','.join(repr(v) for v in virtual),
        'PULSE1_V6_ROUTE_GZ_TIME': repr(stamp),
    })
    print('RECOVER PENDING ONLY:', PENDING_ID, 'ADVANCE WP1', flush=True)
    result = subprocess.run(
        [sys.executable, '-u', str(ROOT / 'scripts/pulse1_v6_single_segment.py'), '--execute'],
        env=env, cwd=ROOT, timeout=120,
    )
    if result.returncode != 0:
        raise RuntimeError('RECOVERY_EXECUTOR_FAILED: DO NOT RETRY')

    rclpy.init(args=[])
    verify = Monitor()
    try:
        verify.wait(lambda: verify.stopped()
                    and verify.value('guard').get('segment_id') == PENDING_ID,
                    'RECOVERY_INDEPENDENT_STOP', timeout=8)
    finally:
        verify.destroy_node()
        rclpy.shutdown()
    print('RECOVERY SUCCESS: FINAL INDEPENDENT STOP', PENDING_ID, flush=True)

if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('STOP WP1 RECOVERY:', exc, file=sys.stderr, flush=True)
        raise SystemExit(1)
