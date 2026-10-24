#!/usr/bin/env python3
"""Read-only regression from recorded WP1 failure. Requires full PULSE-1 repo.

Run on VM: python3 tests/test_pulse1_v6_handoff_wp1_snapshot.py
Does not initialize ROS, rearm, control Gazebo, or send any command.
"""
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts'))
from pulse1_waypoint_plan import load_world
from pulse1_v6_route_handoff import reconcile_route_plan

LOG = ROOT / 'runtime/evidence/v6_video_test_20261023T120812Z/segment_8.log'
assert LOG.is_file(), f'Evidence not found: {LOG}'
s = LOG.read_text()
matches = re.findall(r'SUCCESS V6: pose=\(([^)]+)\)', s)
assert matches, 'Expected final certified ROS odometry in segment_8.log'
ros = tuple(float(x) for x in matches[-1].split(','))
physical = (2.098608010211208, 2.802772822260899, 1.56237)
world = load_world(ROOT / 'runtime/pulse1_world.json')
req, planned, mode = reconcile_route_plan(world, physical, ros,
                      'v6-route-wp1-offline-snapshot', 'LEFT')
assert (mode, req['kind'], req['waypoint_index']) == ('CORNER_BRIDGE', 'ADVANCE', 1), (mode, req)
assert (planned['kind'], planned['waypoint_index']) == ('ADVANCE', 1), planned
assert 0.015 <= req['limit'] <= 0.030, req['limit']
assert req['start_pose'] == list(ros)
print('REAL WORLD + SAFETY CONTRACT: PASS')
print('WP1 BRIDGE: ADVANCE %.6f m -> TURN WP2 on both frames' % req['limit'])
print('NO ROS / NO REARM / NO MOTION')
