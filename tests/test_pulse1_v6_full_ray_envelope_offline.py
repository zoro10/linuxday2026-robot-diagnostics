"""Real S1 full-ray LiDAR geometry: checks behind six sector minima."""
import copy
import math
import random
import sys
import unittest
from pathlib import Path
root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root / 'scripts'))
from pulse1_avoid_sectors import sector_distances
from pulse1_v6_no_return_proof import make_full_scan_rays
from pulse1_v6_safety_contract import (
    turn_clearance_detail, turn_swept_radius_m,
    TURN_LIDARS, TURN_AXLE_X_M, _ray_radial_lower_bound,
)
from pulse1_v6_sensor_preflight import check_sensor_gate


def reading(name, ranges):
    n=len(ranges)
    mn=-math.pi/2
    inc=math.pi/(n-1)
    max_range=12.0 if name=='front' else 6.0
    sectors=sector_distances(ranges,mn,inc,.12,max_range)
    return {'sectors':{k: {'clear':v==math.inf,
                           'distance_m':None if v==math.inf else round(v,3)}
                       for k,v in sectors.items()},
            'full_scan_rays':make_full_scan_rays(name,ranges,mn,inc,.12,max_range)}


def observer(front=None,rear=None):
    front=front if front is not None else [math.inf]*180
    rear=rear if rear is not None else [math.inf]*90
    return {'source':'avoid_observer','mode':'AVOID','state':'SENSOR_READY',
            'motor_commands':False,'odom_ok':True,'lidar_ok':True,
            'rear_lidar_ok':True,'imu_ok':True,
            'readings':{'front':reading('front',front),'rear':reading('rear',rear)}}

class TestFullRayEnvelope(unittest.TestCase):
    def test_real_trace_rear_fringe_ten_hits_safe(self):
        rear=[math.inf]*90
        for i in range(80,90):
            rear[i]=3.004+(i-80)*(2.537-3.004)/9
        o=observer(rear=rear)
        self.assertTrue(all(v['clear'] for v in o['readings']['rear']['sectors'].values()))
        self.assertEqual(turn_clearance_detail(o), (True,'TURN_CLEAR_FULL_RAY_ENVELOPE'))
        self.assertEqual(check_sensor_gate(o,.1,'TURN'),(True,'TURN_CLEAR_FULL_RAY_ENVELOPE'))

    def test_all_infinite_complete_scans_valid(self):
        self.assertTrue(turn_clearance_detail(observer())[0])

    def test_close_hit_in_sector_denied(self):
        f=[math.inf]*180
        f[89]=.3
        ok,reason=turn_clearance_detail(observer(front=f))
        self.assertFalse(ok)
        self.assertIn('FRONT_RAY_89',reason)

    def test_close_hit_outside_sector_denied(self):
        rear=[math.inf]*90
        rear[89]=.13
        ok,reason=turn_clearance_detail(observer(rear=rear))
        self.assertFalse(ok)
        self.assertIn('REAR_RAY_89',reason)

    def test_single_nan_rejected(self):
        r=[math.inf]*90;r[89]=math.nan
        self.assertFalse(turn_clearance_detail(observer(rear=r))[0])

    def test_negative_inf_rejected(self):
        r=[math.inf]*90;r[89]=-math.inf
        self.assertFalse(turn_clearance_detail(observer(rear=r))[0])

    def test_missing_rear_raw_scan_denied(self):
        o=observer();o['readings']['rear']['full_scan_rays']=None
        self.assertFalse(turn_clearance_detail(o)[0])

    def test_wrong_scan_count_denied(self):
        o=observer();o['readings']['rear']['full_scan_rays']['ranges'].pop()
        self.assertFalse(turn_clearance_detail(o)[0])

    def test_metadata_tampered_denied(self):
        o=observer();o['readings']['rear']['full_scan_rays']['angle_inc']*=.8
        self.assertFalse(turn_clearance_detail(o)[0])

    def test_unknown_sector_fails_closed(self):
        o=observer();o['readings']['rear']['sectors']['left']['clear']=None
        self.assertFalse(turn_clearance_detail(o)[0])

    def test_sensor_flag_fail_closed(self):
        o=observer();o['rear_lidar_ok']=False
        self.assertFalse(turn_clearance_detail(o)[0])

    def test_angular_bound_is_conservative_for_sampled_beams(self):
        rng=random.Random(93817)
        for name, (sx,sy,yaw,rmax) in TURN_LIDARS.items():
            n=180 if name=='front' else 90
            half_bin=math.pi/(n-1)/2
            for _ in range(2000):
                angle=rng.uniform(-math.pi/2, math.pi/2)
                measured=rng.uniform(.12,rmax)
                lower=_ray_radial_lower_bound(name,angle,half_bin,measured)
                for da in (-half_bin, 0.0, half_bin):
                    for dr in (-.005, 0.0, .005, .5):
                        r=max(0.0,measured+dr)
                        actual=math.hypot(sx-TURN_AXLE_X_M+r*math.cos(yaw+angle+da),
                                          sy+r*math.sin(yaw+angle+da))
                        self.assertLessEqual(lower,actual+1e-8)

    def test_observer_freshness_via_gate(self):
        self.assertFalse(check_sensor_gate(observer(),.9,'TURN')[0])

    def test_obstacle_near_swept_disk_denied_even_if_behind(self):
        # The rear laser starts INSIDE the swept disk, so a close rear hit
        # must never be accepted merely because the angle is outside sectors.
        r=[math.inf]*90;r[0]=.2
        self.assertFalse(turn_clearance_detail(observer(rear=r))[0])

if __name__ == '__main__':
    unittest.main(verbosity=2)
