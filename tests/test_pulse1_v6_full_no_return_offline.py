"""Fail-closed tests for full-scan LiDAR no-return cases in fixed S1."""
import copy
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))
from pulse1_v6_no_return_proof import (calibrated_s1_files_present, make_no_return_certificate,
                                        verified_no_return_certificate)
from pulse1_v6_safety_contract import (turn_clearance_detail,
                                        turn_swept_radius_m, TURN_LIDARS,
                                        TURN_AXLE_X_M)
from pulse1_v6_sensor_preflight import check_sensor_gate


def proof(sensor, fov=math.pi, rmin=.12, rmax=None, rays=None):
    count, mx = {'front':(180,12.), 'rear':(90,6.)}[sensor]
    vals = [float('inf')]*count if rays is None else rays
    return make_no_return_certificate(sensor, vals, -fov/2,
                    fov/(count-1), rmin, mx if rmax is None else rmax)


def observer():
    sectors = {i:{'distance_m':None,'clear':True}
               for i in ('front','left','right')}
    return {'state':'SENSOR_READY','source':'avoid_observer',
            'mode':'AVOID','motor_commands':False,
            'odom_ok':True,'lidar_ok':True,'rear_lidar_ok':True,'imu_ok':True,
            'readings':{sensor:{'sectors':copy.deepcopy(sectors),
                      'full_scan_no_return':proof(sensor)}
                      for sensor in ('front','rear')}}


class FullNoReturnTest(unittest.TestCase):
    def test_fixed_s1_calibration_present(self):
        self.assertTrue(calibrated_s1_files_present())
    def test_valid_complete_scans_are_clear(self):
        o=observer()
        self.assertEqual(turn_clearance_detail(o),
                         (True,'TURN_CLEAR_FULL_SCAN_NO_RETURN'))
        self.assertEqual(check_sensor_gate(o,.1,'TURN'),
                         (True,'TURN_CLEAR_FULL_SCAN_NO_RETURN'))
    def test_bare_sector_claims_denied(self):
        o=observer(); del o['readings']['front']['full_scan_no_return']
        self.assertEqual(turn_clearance_detail(o),
                         (False,'TURN_ALL_SECTORS_NO_FINITE_RETURN'))
    def test_nan_ray_never_certifies(self):
        r=[float('inf')]*180; r[20]=float('nan')
        self.assertIsNone(proof('front',rays=r))
    def test_finite_ray_outside_sectors_never_certifies(self):
        r=[float('inf')]*180; r[0]=2.0
        self.assertIsNone(proof('front',rays=r))
    def test_bad_range_max_never_certifies(self):
        self.assertIsNone(proof('rear',rmax=5.0))
    def test_narrow_fov_never_certifies(self):
        self.assertIsNone(proof('front',fov=2.5))
    def test_missing_rear_certificate_denied(self):
        o=observer();o['readings']['rear']['full_scan_no_return']=None
        self.assertFalse(turn_clearance_detail(o)[0])
    def test_invalid_sector_denied_even_with_proof(self):
        o=observer();o['readings']['rear']['sectors']['left']={'clear':None,'distance_m':None}
        self.assertFalse(turn_clearance_detail(o)[0])
    def test_unknown_sensor_flag_denied(self):
        o=observer();o['lidar_ok']=False
        self.assertFalse(turn_clearance_detail(o)[0])
    def test_stale_observer_denied_by_gate(self):
        o=observer()
        self.assertFalse(check_sensor_gate(o,.9,'TURN')[0])
    def test_small_finite_hit_still_denied(self):
        o=observer();o['readings']['front']['sectors']['front']={'distance_m':.5,'clear':False}
        self.assertFalse(turn_clearance_detail(o)[0])
    def test_both_ranges_cover_full_swept_disk(self):
        radius=turn_swept_radius_m()
        for name,(x,y,yaw,maximum) in TURN_LIDARS.items():
            self.assertGreater(maximum, radius+math.hypot(x-TURN_AXLE_X_M,y))
    def test_metadata_corrupt_denied(self):
        o=observer();o['readings']['front']['full_scan_no_return']['range_max']=100.
        self.assertFalse(turn_clearance_detail(o)[0])

if __name__=='__main__':unittest.main(verbosity=2)
