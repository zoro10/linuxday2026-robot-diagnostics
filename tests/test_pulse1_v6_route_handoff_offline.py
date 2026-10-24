#!/usr/bin/env python3
"""Standalone PURE geometry unit tests with explicit dependency doubles.

Run: python3 tests/test_pulse1_v6_route_handoff_offline.py
These tests never start ROS or authorize motion. They do NOT certify Gazebo.
"""
import ast
import math
import os
import tempfile
from pathlib import Path
import sys
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))

WAYPOINTS = [(2.1, 0.0), (2.1, 2.83143), (4.0, 2.83143),
             (4.0, 0.5), (5.0, 0.5)]

def build_route(world, side):
    return {'waypoints': WAYPOINTS, 'inflated_obstacle_bounds': []}

def valid_request(req):
    return (isinstance(req, dict)
            and req.get('kind') in ('ADVANCE', 'TURN')
            and isinstance(req.get('waypoint_index'), int)
            and isinstance(req.get('limit'), (float, int))
            and math.isfinite(req['limit']) and 0 < req['limit'] <= .21
            and len(req.get('start_pose', [])) == 3)

def pose_valid(p):
    return (isinstance(p, (list, tuple)) and len(p) == 3
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in p))

# Deliberately replace imports in THIS PROCESS to test the actual new pure
# planning module independently of user's ROS / asset files.
wp = types.ModuleType('pulse1_waypoint_plan')
wp.build_route = build_route
wp.segment_hits_aabb = lambda *args: False
sys.modules[wp.__name__] = wp
safety = types.ModuleType('pulse1_v6_safety_contract')
safety.pose_valid = pose_valid
safety.valid_request = valid_request
safety.MAX_FORWARD_M = .1
safety.MAX_LINEAR_M_S = .1
sys.modules[safety.__name__] = safety
turn = types.ModuleType('pulse1_v6_turn_policy')
turn.TURN_LIMIT = .20944
turn.TURN_SPEED = .20
sys.modules[turn.__name__] = turn

from pulse1_v6_route_handoff import reconcile_route_plan
from pulse1_v6_route_policy import next_route_request

ID = 'v6-test-geometry-only'

class RouteHandoffOfflineTest(unittest.TestCase):
    def check_bridge(self, gz, odom, expected_wp, turn_dir):
        req, plan, mode = reconcile_route_plan(None, gz, odom, ID)
        self.assertEqual(mode, 'CORNER_BRIDGE')
        self.assertEqual(req['kind'], 'ADVANCE')
        self.assertEqual(req['waypoint_index'], expected_wp - 1)
        self.assertEqual(req['start_pose'], list(odom))
        self.assertGreaterEqual(req['limit'], .020)
        self.assertLessEqual(req['limit'], .10)
        for pose in (gz, odom):
            p = (pose[0] + req['limit'] * math.cos(pose[2]),
                 pose[1] + req['limit'] * math.sin(pose[2]), pose[2])
            nxt = next_route_request(None, p, ID)
            self.assertEqual((nxt['kind'], nxt['waypoint_index'], nxt['direction']),
                             ('TURN', expected_wp, turn_dir))

    def test_wp1_right_odom_ahead(self):
        self.check_bridge((2.1,2.80277,1.56237),
                          (2.11324,2.82050,1.56252),2,'RIGHT')

    def test_wp1_right_gazebo_ahead(self):
        self.check_bridge((2.11324,2.82050,1.56252),
                          (2.1,2.80277,1.56237),2,'RIGHT')

    def test_wp2_right_odom_ahead(self):
        self.check_bridge((3.97134,2.83143,0.),
                          (3.98907,2.81819,.00015),3,'RIGHT')

    def test_wp3_left_odom_ahead(self):
        self.check_bridge((4.0,.52866,-math.pi/2),
                          (4.01324,.51093,-math.pi/2+.00015),4,'LEFT')

    def test_plain_advance_agrees(self):
        req, old, mode = reconcile_route_plan(None,
                        (2.1,1.0,math.pi/2), (2.1,1.01,math.pi/2), ID)
        self.assertEqual((mode,req['kind'],req['waypoint_index']),('AGREED','ADVANCE',1))

    def test_plain_turn_agrees(self):
        req, old, mode = reconcile_route_plan(None,
                        (2.1,2.821,math.pi/2),(2.11,2.82,math.pi/2),ID)
        self.assertEqual((mode,req['kind'],req['waypoint_index']),('AGREED','TURN',2))

    def test_full_heading_axle_compensation_wp1_wp2_wp3(self):
        # Extract the ACTUAL production implementation without importing ROS.
        source = ast.parse((Path(ROOT) / 'scripts/pulse1_v6_sequencer.py').read_text())
        func = next(n for n in source.body if isinstance(n, ast.FunctionDef)
                    and n.name == 'v6_corner_planning_pose')
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp) / 'simulation/models/pulse1/model_demo2.sdf'
            model.parent.mkdir(parents=True)
            model.write_text('<model name="pulse1">'
                '<link name="left_wheel"><pose>0.554 0.625 0 0 0 0</pose></link>'
                '<link name="right_wheel"><pose>0.554 -0.625 0 0 0 0</pose></link>'
                '</model>')
            ns = {'ROOT': Path(tmp), 'math': math,
                  'next_route_request': next_route_request}
            exec(compile(ast.Module(body=[func], type_ignores=[]),
                         '<extracted-real-sequencer>', 'exec'), ns)
            fn = ns['v6_corner_planning_pose']
            for target, heading in ((WAYPOINTS[1],math.pi/2),
                                    (WAYPOINTS[2],0.0),
                                    (WAYPOINTS[2],-math.pi/2),
                                    (WAYPOINTS[3],-math.pi/2)):
                origin = (target[0] - .554*(math.cos(heading)-1),
                          target[1] - .554*math.sin(heading), heading)
                corrected = fn(origin, None, 'LEFT')
                self.assertAlmostEqual(corrected[0],target[0],places=6)
                self.assertAlmostEqual(corrected[1],target[1],places=6)

    def test_large_drift_stops(self):
        with self.assertRaisesRegex(ValueError,'HANDOFF_FRAME_DISAGREEMENT'):
            reconcile_route_plan(None,(2.1,2.8,math.pi/2),
                                 (2.2,2.8,math.pi/2),ID)

    def test_large_heading_drift_stops(self):
        with self.assertRaisesRegex(ValueError,'HANDOFF_FRAME_DISAGREEMENT'):
            reconcile_route_plan(None,(2.1,2.8,math.pi/2),
                                 (2.1,2.8,math.pi/2+.1),ID)

if __name__ == '__main__':
    unittest.main(verbosity=2)
