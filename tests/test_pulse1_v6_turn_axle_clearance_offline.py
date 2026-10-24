#!/usr/bin/env python3
"""S1 geometric calibration and conservative turning-contract tests. Pure offline."""
import math
import random
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts'))
from pulse1_v6_safety_contract import (
    TURN_AXLE_X_M, TURN_CHASSIS_XMIN_M, TURN_CHASSIS_XMAX_M,
    TURN_CHASSIS_HALF_WIDTH_M, TURN_WHEEL_Y_M, TURN_WHEEL_RADIUS_M,
    TURN_CASTER_X_M, TURN_CASTER_RADIUS_M, TURN_LIDARS,
    TURN_SECTOR_ANGLES, turn_swept_radius_m, _sector_radial_lower_bound,
    turn_clear, turn_clearance_detail,
)
from pulse1_v6_sensor_preflight import check_sensor_gate
from pulse1_avoid_sectors import SECTORS


def obs(front=None, rear=None):
    """Pass an optional {'front': d, 'left': d, 'right': d}; None means no return."""
    def s(values):
        values = values or {}
        return {name: {'clear': values.get(name) is None,
                       'distance_m': values.get(name)}
                for name in ('front', 'left', 'right')}
    return {'source': 'avoid_observer', 'mode': 'AVOID',
            'state': 'SENSOR_READY', 'motor_commands': False,
            'odom_ok': True, 'lidar_ok': True, 'rear_lidar_ok': True, 'imu_ok': True,
            'readings': {'front': {'sectors': s(front)},
                         'rear': {'sectors': s(rear)}}}


class GeometricTurningTest(unittest.TestCase):
    def test_sdf_calibration_and_sector_definitions(self):
        root = ET.parse(ROOT / 'simulation/models/pulse1/model_demo2.sdf').getroot()
        chassis = root.find(".//link[@name='chassis']")
        self.assertIsNotNone(chassis)
        cx = float(chassis.find('pose').text.split()[0])
        dims = [float(v) for v in chassis.find('collision/geometry/box/size').text.split()]
        self.assertAlmostEqual(TURN_CHASSIS_XMIN_M, cx - dims[0]/2, places=6)
        self.assertAlmostEqual(TURN_CHASSIS_XMAX_M, cx + dims[0]/2, places=6)
        self.assertAlmostEqual(TURN_CHASSIS_HALF_WIDTH_M, dims[1]/2, places=6)
        left = root.find(".//link[@name='left_wheel']")
        right = root.find(".//link[@name='right_wheel']")
        for wheel in (left, right):
            x,y = (float(v) for v in wheel.find('pose').text.split()[:2])
            self.assertAlmostEqual(TURN_AXLE_X_M, x, delta=1e-5)
            self.assertAlmostEqual(TURN_WHEEL_Y_M, abs(y), places=5)
            self.assertAlmostEqual(TURN_WHEEL_RADIUS_M,
                                   float(wheel.find('collision/geometry/sphere/radius').text), places=6)
        caster = root.find(".//link[@name='caster']")
        self.assertAlmostEqual(TURN_CASTER_X_M,float(caster.find('pose').text.split()[0]), places=6)
        self.assertAlmostEqual(TURN_CASTER_RADIUS_M,
            float(caster.find('collision/geometry/sphere/radius').text), places=6)
        for name in ('front', 'rear'):
            sensor = root.find(f".//sensor[@name='pulse1_lidar{'_rear' if name == 'rear' else ''}']")
            self.assertIsNotNone(sensor)
            vals = [float(v) for v in sensor.find('pose').text.split()]
            sx,sy,yaw,maximum = TURN_LIDARS[name]
            self.assertAlmostEqual(sx, cx+vals[0], places=6)
            self.assertAlmostEqual(sy, vals[1], places=6)
            self.assertAlmostEqual(yaw, vals[5], places=6)
            self.assertAlmostEqual(maximum, float(sensor.find('ray/range/max').text), places=6)
        self.assertEqual(SECTORS, TURN_SECTOR_ANGLES)

    def test_rigorous_envelope_includes_all_collision_shapes(self):
        self.assertAlmostEqual(turn_swept_radius_m(),1.9829629,places=5)

    def test_real_report_1_990_rear_left_outside_full_turn_envelope(self):
        o = obs(rear={'left':1.990})
        lower = _sector_radial_lower_bound('rear','left',1.990)
        self.assertAlmostEqual(lower,3.040724,places=5)
        self.assertGreater(lower,turn_swept_radius_m()+.005)
        self.assertEqual(turn_clearance_detail(o), (True, 'TURN_CLEAR'))
        self.assertTrue(turn_clear(o))
        self.assertEqual(check_sensor_gate(o,.1,'TURN'),(True,'TURN_CLEAR'))

    def test_obstacle_world_static_clearance_at_wp1(self):
        root = ET.parse(ROOT / 'simulation/worlds/pulse1_s1.sdf').getroot()
        model = root.find(".//model[@name='pulse1_s1_obstacle']")
        x,y = (float(v) for v in model.find('pose').text.split()[:2])
        size = [float(v) for v in model.find('.//collision/geometry/box/size').text.split()]
        xmin,xmax = x-size[0]/2,x+size[0]/2
        ymin,ymax = y-size[1]/2,y+size[1]/2
        px=2.6483+TURN_AXLE_X_M*math.cos(1.56237)
        py=2.25851+TURN_AXLE_X_M*math.sin(1.56237)
        dx=max(xmin-px,0,px-xmax)
        dy=max(ymin-py,0,py-ymax)
        self.assertGreater(math.hypot(dx,dy), turn_swept_radius_m()+.005)

    def test_close_front_hit_denied(self):
        o = obs(front={'front':1.5})
        self.assertFalse(turn_clear(o))
        self.assertIn('TURN_FRONT_FRONT_TOO_CLOSE',check_sensor_gate(o,.1,'TURN')[1])

    def test_close_rear_sector_denied(self):
        o = obs(rear={'left':0.13})
        self.assertFalse(turn_clear(o))
        self.assertIn('TURN_REAR_LEFT_TOO_CLOSE',check_sensor_gate(o,.1,'TURN')[1])

    def test_unknown_and_all_infinite_fail_closed(self):
        self.assertFalse(turn_clear(obs()))
        o = obs(rear={'left':1.99})
        del o['readings']['rear']['sectors']['right']
        self.assertFalse(turn_clear(o))
        self.assertEqual(check_sensor_gate(o,.1,'TURN')[1],'TURN_REAR_RIGHT_MISSING')

    def test_invalid_and_stale_fail_closed(self):
        o = obs(rear={'left':7.0})  # beyond this LiDAR's physical 6m maximum
        self.assertFalse(turn_clear(o))
        self.assertIn('INVALID',check_sensor_gate(o,.1,'TURN')[1])
        o = obs(rear={'left':1.99})
        self.assertFalse(check_sensor_gate(o,.9,'TURN')[0])
        o['readings']['rear']['sectors']['left']['clear']=True
        self.assertFalse(turn_clear(o))

    def test_analytic_lower_bound_conservative_against_sampled_beams(self):
        rng=random.Random(0xA113)
        for sensor in ('front','rear'):
            sx,sy,yaw,max_range=TURN_LIDARS[sensor]
            for sector,(lo,hi) in TURN_SECTOR_ANGLES.items():
                for d in (.12,.5,1.,1.99,2.5,3.,5.):
                    if d>max_range: continue
                    lower = _sector_radial_lower_bound(sensor,sector,d)
                    for _ in range(100):
                        angle=yaw+rng.uniform(lo,hi)
                        radius=rng.uniform(d,max_range)
                        hitx=sx+radius*math.cos(angle)-TURN_AXLE_X_M
                        hity=sy+radius*math.sin(angle)
                        self.assertGreaterEqual(math.hypot(hitx,hity)+1e-9,lower)


if __name__=='__main__':
    unittest.main(verbosity=2)
