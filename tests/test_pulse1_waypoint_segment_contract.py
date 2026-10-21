import copy
import math
import unittest

from pulse1_waypoint_plan import build_route, WaypointShadow
from pulse1_waypoint_segment_contract import (
    propose_segment, MAX_FORWARD_SEGMENT_M, MAX_TURN_SEGMENT_RAD
)

WORLD = {'scenario': 's1', 'navigation_mode': 'AVOID',
         'obstacles': [{'name': 'pulse1_s1_obstacle', 'x': 5.0, 'y': 0.0,
                        'size_x': 1.0, 'size_y': 1.4}]}


def observer(x=0, y=0, yaw=0, front=4.2):
    def sector():
        return {'clear': False, 'distance_m': front}
    return {'source': 'avoid_observer', 'mode': 'AVOID', 'state': 'SENSOR_READY',
            'motor_commands': False,
            'odom_ok': True, 'lidar_ok': True, 'rear_lidar_ok': True, 'imu_ok': True,
            'readings': {'odom': {'x': x, 'y': y, 'yaw_rad': yaw},
                         'front': {'sectors': {'front': sector(), 'left': sector(), 'right': sector()}},
                         'rear': {'sectors': {'front': sector(), 'left': sector(), 'right': sector()}}}}


def sample(side='LEFT', x=0, y=0, yaw=0):
    route = build_route(WORLD, side)
    data = observer(x,y,yaw)
    wp = WaypointShadow(route).evaluate(data, 0.1)
    return {'route': route, 'waypoint': wp, 'waypoint_age_s': 0.1,
            'observer': data, 'observer_age_s': 0.1,
            'guard': {'source': 'motor_guard_live', 'mode': 'AVOID', 'motor_commands': True,
                      'guard_state': 'STALE_REQUEST', 'permitted_linear_m_s': 0.0,
                      'permitted_angular_rad_s': 0.0}, 'guard_age_s': 0.1,
            'cmd': (0., 0.), 'cmd_age_s': 0.1,
            'world_paused': True, 'world_age_s': 0.1}


class TestSegmentShadow(unittest.TestCase):
    def candidate(self, overrides=None, **kwargs):
        data = sample(**kwargs)
        if overrides:
            for key, value in overrides.items():
                data[key] = value
        return propose_segment(**data)

    def assert_non_motor(self, result):
        self.assertIs(result['motor_commands'], False)
        self.assertIs(result['motion_authorized'], False)
        self.assertEqual(result['proposed_linear_m_s'], 0)
        self.assertEqual(result['proposed_angular_rad_s'], 0)

    def test_first_20cm_preview_only(self):
        s = self.candidate()
        self.assertEqual(s['state'], 'CANDIDATE')
        self.assertEqual(s['candidate']['kind'], 'ADVANCE')
        self.assertEqual(s['candidate']['max_distance_m'], MAX_FORWARD_SEGMENT_M)
        self.assertTrue(s['candidate']['requires_independent_stop_ack'])
        self.assert_non_motor(s)

    def test_left_turn_bounded_to_15_degrees(self):
        s = self.candidate(x=2.368566677973659)
        self.assertEqual(s['state'], 'CANDIDATE')
        self.assertEqual(s['candidate']['kind'], 'TURN')
        self.assertEqual(s['candidate']['direction'], 'LEFT')
        self.assertLessEqual(s['candidate']['max_angle_rad'], MAX_TURN_SEGMENT_RAD)
        self.assert_non_motor(s)

    def test_right_turn(self):
        s = self.candidate(side='RIGHT', x=2.368566677973659)
        self.assertEqual(s['candidate']['direction'], 'RIGHT')

    def test_stale_observer_fails_closed(self):
        self.assertEqual(self.candidate({'observer_age_s': 1.0})['reason'], 'OBSERVER_UNAVAILABLE')

    def test_world_must_be_paused(self):
        self.assertEqual(self.candidate({'world_paused': False})['reason'], 'WORLD_NOT_PAUSED')

    def test_guard_must_be_zero(self):
        d = sample(); d['guard']['permitted_linear_m_s'] = .1
        self.assertEqual(propose_segment(**d)['reason'], 'GUARD_NOT_ZERO')

    def test_cmd_zero_and_fresh(self):
        self.assertEqual(self.candidate({'cmd': (.1,0.)})['reason'], 'CMD_NOT_ZERO')
        self.assertEqual(self.candidate({'cmd_age_s': .7})['reason'], 'CMD_NOT_ZERO')

    def test_bad_waypoint_source(self):
        d=sample(); d['waypoint']['source'] = 'spoof'
        self.assertEqual(propose_segment(**d)['reason'], 'INVALID_WAYPOINT_SOURCE')

    def test_waypoint_replay(self):
        self.assertEqual(self.candidate({'waypoint_age_s': 1.0})['reason'], 'WAYPOINT_STALE')

    def test_target_tampering(self):
        d=sample(); d['waypoint']['target']=[9,1]
        self.assertEqual(propose_segment(**d)['reason'], 'TARGET_CHANGED')

    def test_pose_disagree(self):
        d=sample(); d['waypoint']['pose']=[1.0,0.0,0.0]
        self.assertEqual(propose_segment(**d)['reason'], 'POSES_DISAGREE')

    def test_front_too_close(self):
        d=sample(); d['observer'] = observer(front=1.3)
        self.assertEqual(propose_segment(**d)['reason'], 'FRONT_NOT_CLEAR')

    def test_no_turn_without_sector_clearance(self):
        d=sample(x=2.368566677973659)
        del d['observer']['readings']['rear']['sectors']['left']
        self.assertEqual(propose_segment(**d)['reason'], 'TURN_CLEARANCE_UNAVAILABLE')

    def test_turn_clearance_too_small(self):
        d=sample(x=2.368566677973659)
        d['observer'] = observer(x=2.368566677973659, front=2.0)
        self.assertEqual(propose_segment(**d)['reason'], 'TURN_CLEARANCE_TOO_SMALL')

    def test_nonfinite_pose_denied(self):
        d=sample(); d['waypoint']['pose']=[math.nan,0,0]
        self.assertEqual(propose_segment(**d)['reason'], 'INVALID_POSE')

    def test_invalid_route_denied(self):
        d=sample(); d['route']['side']='AUTO'
        self.assertEqual(propose_segment(**d)['reason'], 'INVALID_ROUTE')

    def test_untrusted_sensors(self):
        d=sample(); d['observer']['imu_ok'] = False
        self.assertEqual(propose_segment(**d)['reason'], 'OBSERVER_UNAVAILABLE')

    def test_occupied_keepout_rejected(self):
        d=sample(x=5)
        # Shadow at x=5 will be PLAN for first waypoint.
        self.assertEqual(propose_segment(**d)['reason'], 'INSIDE_KEEPOUT')


if __name__=='__main__':
    unittest.main()
