import copy
import math
import unittest

from pulse1_waypoint_plan import (
    GOAL, START, Rectangle, WaypointShadow, build_route,
    front_verified, observer_pose, s1_obstacle, segment_hits_aabb,
    valid_observer,
)

WORLD = {
    'scenario': 's1',
    'navigation_mode': 'AVOID',
    'obstacles': [{'name': 'pulse1_s1_obstacle', 'x': 5.0, 'y': 0.0,
                   'size_x': 1.0, 'size_y': 1.4}],
}


def reading(x=0.0, y=0.0, yaw=0.0, *, clearance=4.2):
    return {
        'source': 'avoid_observer', 'mode': 'AVOID', 'state': 'SENSOR_READY',
        'motor_commands': False, 'odom_ok': True, 'lidar_ok': True,
        'rear_lidar_ok': True, 'imu_ok': True,
        'readings': {
            'odom': {'x': x, 'y': y, 'yaw_rad': yaw},
            'front': {'sectors': {'front': {
                'clear': clearance == math.inf,
                'distance_m': None if clearance == math.inf else clearance,
            }}},
        },
    }


class TestWaypointS1(unittest.TestCase):
    def test_both_routes_clear_conservative_keepout(self):
        obstacle = s1_obstacle(WORLD)
        for side in ('LEFT', 'RIGHT'):
            with self.subTest(side=side):
                route = build_route(WORLD, side)
                inflated = obstacle.bounds(route['turning_envelope_m'])
                # Rounded reported radius is for display only. Actual planner
                # used full float. Use a tiny lower radius when testing output.
                inflated = obstacle.bounds(route['turning_envelope_m'] - .0001)
                points = [START] + [tuple(p) for p in route['waypoints']]
                self.assertEqual(points[-1], GOAL)
                self.assertEqual(len(points), 6)
                self.assertTrue(all(not segment_hits_aabb(a, b, inflated)
                                    for a, b in zip(points, points[1:])))
                self.assertFalse(route['motion_authorized'])
                self.assertGreater(route['path_length_m'], 10)

    def test_not_just_half_width_clearance(self):
        r = build_route(WORLD, 'LEFT')
        self.assertGreater(r['waypoints'][1][1], 2.7)
        self.assertLess(r['waypoints'][0][0], 2.5)

    def test_unknown_world_fails_closed(self):
        for key, value in [('scenario', 'demo1'), ('navigation_mode', 'STOP')]:
            data = copy.deepcopy(WORLD); data[key] = value
            with self.assertRaises(ValueError):
                build_route(data, 'LEFT')
        data = copy.deepcopy(WORLD); data['obstacles'][0]['x'] = 4.0
        with self.assertRaises(ValueError):
            build_route(data, 'LEFT')
        with self.assertRaises(ValueError):
            build_route(WORLD, 'AUTO')

    def test_geometry_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            Rectangle(5, 0, math.nan, 2)
        with self.assertRaises(ValueError):
            Rectangle(5, 0, 0, 2)
        self.assertTrue(segment_hits_aabb((0, 0), (10, 0), (4, 6, -1, 1)))
        self.assertFalse(segment_hits_aabb((0, 3), (10, 3), (4, 6, -1, 1)))

    def test_shadow_waits_stale_or_bad_sensors(self):
        shadow = WaypointShadow(build_route(WORLD, 'LEFT'))
        data = reading()
        for age in (None, -1, 0.61, float('nan')):
            self.assertEqual(shadow.evaluate(data, age)['state'], 'HOLD')
        data['imu_ok'] = False
        self.assertEqual(shadow.evaluate(data, 0.1)['state'], 'HOLD')
        self.assertEqual(shadow.index, 0)

    def test_shadow_rejects_invalid_pose(self):
        shadow = WaypointShadow(build_route(WORLD, 'LEFT'))
        data = reading(); data['readings']['odom']['yaw_rad'] = math.inf
        result = shadow.evaluate(data, 0.1)
        self.assertEqual(result['reason'], 'INVALID_POSE')
        self.assertEqual(result['proposed_linear_m_s'], 0)
        self.assertFalse(result['motor_commands'])

    def test_front_unknown_and_close_stop(self):
        shadow = WaypointShadow(build_route(WORLD, 'LEFT'))
        data = reading()
        data['readings']['front']['sectors']['front'] = {'distance_m': None, 'clear': False}
        self.assertEqual(shadow.evaluate(data, 0.0)['reason'], 'UNKNOWN_FRONT')
        data = reading(clearance=1.1)
        self.assertEqual(shadow.evaluate(data, 0.0)['reason'], 'OBSTACLE_TOO_CLOSE')

    def test_inf_clear_valid(self):
        self.assertEqual(front_verified(reading(clearance=math.inf)), (True, math.inf))

    def test_waypoints_advance_sequentially_without_moving(self):
        route = build_route(WORLD, 'LEFT')
        shadow = WaypointShadow(route)
        first = route['waypoints'][0]
        r = shadow.evaluate(reading(*first), 0.1)
        self.assertEqual(shadow.index, 1)
        self.assertEqual(r['waypoint_index'], 1)
        self.assertEqual(r['suggested_action'], 'TURN_LEFT')
        self.assertFalse(r['motion_authorized'])
        # A pose that jumped to the last point cannot skip waypoint #2.
        shadow.evaluate(reading(10, 0), 0.1)
        self.assertEqual(shadow.index, 1)
        for point in route['waypoints'][1:]:
            shadow.evaluate(reading(*point), 0.1)
        self.assertTrue(shadow.arrived)
        self.assertEqual(shadow.evaluate(None, None)['state'], 'ARRIVED')

    def test_right_turn_candidate(self):
        shadow = WaypointShadow(build_route(WORLD, 'RIGHT'))
        x, y = shadow.route['waypoints'][0]
        r = shadow.evaluate(reading(x, y, 0), 0)
        self.assertEqual(r['suggested_action'], 'TURN_RIGHT')


if __name__ == '__main__':
    unittest.main()
