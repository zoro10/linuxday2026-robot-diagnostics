"""Offline fail-closed regression tests for a bounded (<=15 deg) V6 TURN.

No ROS, motion, rearm, Gazebo, or real-world authorizations.
"""
import math
import unittest

from pulse1_avoid_sectors import sector_distances
from pulse1_v6_no_return_proof import make_full_scan_rays
from pulse1_v6_safety_contract import (
    _bounded_turn_swept_aabb,
    MAX_TURN_RAD, MAX_OVERSHOOT_RAD,
    TURN_AXLE_X_M, TURN_CASTER_RADIUS_M, TURN_CASTER_X_M,
    TURN_CHASSIS_XMIN_M, TURN_CHASSIS_XMAX_M, TURN_CHASSIS_HALF_WIDTH_M,
    TURN_WHEEL_RADIUS_M, TURN_WHEEL_Y_M,
    TURN_NUMERIC_MARGIN_M, TURN_SAFETY_MARGIN_M,
    turn_clearance_detail,
)


def scan(name, finite_hits=None):
    count = 180 if name == 'front' else 90
    maximum = 12.0 if name == 'front' else 6.0
    rays = [math.inf] * count
    for index, value in (finite_hits or {}).items():
        rays[index] = value
    a = -math.pi/2
    inc = math.pi/(count-1)
    sectors = sector_distances(rays, a, inc, .12, maximum)
    return {
        'sectors': {
            key: {'clear': value == math.inf,
                  'distance_m': None if value == math.inf else round(value,3)}
            for key, value in sectors.items()
        },
        'full_scan_rays': make_full_scan_rays(name, rays, a, inc, .12, maximum),
    }


def observer(front=None, rear=None):
    return {
        'source': 'avoid_observer', 'mode': 'AVOID',
        'state': 'SENSOR_READY', 'motor_commands': False,
        'odom_ok': True, 'lidar_ok': True,
        'rear_lidar_ok': True, 'imu_ok': True,
        'readings': {
            'front': scan('front', front),
            'rear': scan('rear', rear),
        },
    }


class TestBoundedArc(unittest.TestCase):
    def test_world_s1_false_positive_at_front_ray_72(self):
        # Measured previous rejection: a 2.234 m return at -17.6 deg.
        # It's outside the small turn's protected space but within 360 disk.
        ok, reason = turn_clearance_detail(observer(front={72: 2.234}))
        self.assertTrue(ok, reason)

    def test_front_obstacle_occluding_protected_space_denied(self):
        ok, reason = turn_clearance_detail(observer(front={89: .3}))
        self.assertFalse(ok, reason)
        self.assertIn('ARC_VISIBILITY_BLOCKED', reason)

    def test_near_rear_fringe_obstacle_denied(self):
        ok, reason = turn_clearance_detail(observer(rear={0: .2}))
        self.assertFalse(ok, reason)
        self.assertIn('REAR_RAY_0', reason)

    def test_full_scans_no_returns_are_safe(self):
        self.assertTrue(turn_clearance_detail(observer())[0])

    def test_missing_proof_denied(self):
        o = observer()
        o['readings']['rear']['full_scan_rays'] = None
        self.assertFalse(turn_clearance_detail(o)[0])

    def test_unknown_observer_denied(self):
        o = observer()
        o['state'] = 'WAIT_SENSORS'
        self.assertFalse(turn_clearance_detail(o)[0])

    def test_swept_box_contains_all_shapes_both_turn_directions(self):
        x0,x1,y0,y1 = _bounded_turn_swept_aabb()
        samples = [(x,y,0.0)
            for x in (TURN_CHASSIS_XMIN_M,TURN_CHASSIS_XMAX_M)
            for y in (-TURN_CHASSIS_HALF_WIDTH_M, TURN_CHASSIS_HALF_WIDTH_M)]
        samples += [(TURN_AXLE_X_M, sy * TURN_WHEEL_Y_M, TURN_WHEEL_RADIUS_M)
                    for sy in (-1,+1)]
        samples.append((TURN_CASTER_X_M,0.0,TURN_CASTER_RADIUS_M))
        angle = MAX_TURN_RAD + MAX_OVERSHOOT_RAD
        for step in range(501):
            theta = -angle + 2*angle*step/500
            for cx,cy,radius in samples:
                dx = cx - TURN_AXLE_X_M
                x=TURN_AXLE_X_M + dx*math.cos(theta)-cy*math.sin(theta)
                y=dx*math.sin(theta)+cy*math.cos(theta)
                allowance = radius + TURN_SAFETY_MARGIN_M + TURN_NUMERIC_MARGIN_M
                self.assertGreaterEqual(x-allowance, x0-1e-10)
                self.assertLessEqual(x+allowance, x1+1e-10)
                self.assertGreaterEqual(y-allowance, y0-1e-10)
                self.assertLessEqual(y+allowance, y1+1e-10)

    def test_static_s1_front_obstacle_outside_protected_arc(self):
        # S1 obstacle from immutable SDF: centre x=5, size x=1, front x=4.5.
        # Actual Gazebo model-origin x at WP1 check: 2.07415 m.
        _, x_max, _, _ = _bounded_turn_swept_aabb()
        self.assertGreater(4.5-(2.07415+x_max), 1.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
