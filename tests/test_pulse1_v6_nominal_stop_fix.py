import copy
import unittest
from pulse1_v6_ros_safety_policy import nominal_v6_stop

R = {
    'source': 'v6_segment_executor', 'mode': 'AVOID',
    'segment_id': 'seg_123456789', 'waypoint_index': 0,
    'kind': 'ADVANCE', 'motor_commands': False, 'phase': 'DRIVE',
    'start_pose': [0.0,0.0,0.0], 'target': [2.3,0.0],
    'limit': 0.2, 'direction': '',
    'proposed_linear_m_s': .1, 'proposed_angular_rad_s': 0.0,
    'odom_ok': True, 'lidar_ok': True, 'rear_lidar_ok': True, 'imu_ok': True,
}
W = {'source': 'v6_geofence_watchdog', 'mode': 'AVOID', 'armed': True,
     'motor_commands': False, 'action':'STOPPING', 'reason':'TARGET_REACHED',
     'segment_id':'seg_123456789', 'kind':'ADVANCE', 'waypoint_index':0}

def check(w=W,r=R,age=.01,segment_id='seg_123456789'):
    return nominal_v6_stop(watchdog=w,watchdog_age_s=age,request=r,active_segment_id=segment_id)

class TestNominalStop(unittest.TestCase):
    def test_valid_nominal_stop(self): self.assertTrue(check())
    def test_valid_hold_stop(self):
        r=copy.deepcopy(R); r.update(phase='HOLD',reason='SEGMENT_TARGET_REACHED',proposed_linear_m_s=0.0)
        self.assertTrue(check(r=r))
    def test_null_data(self):
        for w,r in ((None,R),(W,None),(None,None),([],R), (W,[])):
            self.assertFalse(check(w=w,r=r))
    def test_lost_lease(self): self.assertFalse(check(age=.251))
    def test_lost_guard_ident(self): self.assertFalse(check(segment_id='seg_00000000'))
    def test_wrong_watchdog_ident(self):
        w=dict(W,segment_id='seg_00000000'); self.assertFalse(check(w=w))
    def test_wrong_kind(self):
        w=dict(W,kind='TURN'); self.assertFalse(check(w=w))
    def test_wrong_index(self):
        w=dict(W,waypoint_index=1); self.assertFalse(check(w=w))
    def test_unarmed(self):
        w=dict(W,armed=False); self.assertFalse(check(w=w))
    def test_no_premature_stop(self):
        for action in ('MONITOR','WAIT','PAUSE','COMPLETE'):
            self.assertFalse(check(w=dict(W,action=action)))
    def test_wrong_reason(self): self.assertFalse(check(w=dict(W,reason='SENSOR_STALE')))
    def test_malformed_request(self):
        self.assertFalse(check(r=dict(R,limit=1.0)))
        self.assertFalse(check(r=dict(R,phase='HOLD')))
    def test_watchdog_claim_motor(self): self.assertFalse(check(w=dict(W,motor_commands=True)))
    def test_watchdog_wrong_source(self): self.assertFalse(check(w=dict(W,source='old_geofence')))
    def test_request_wrong_source(self): self.assertFalse(check(r=dict(R,source='v5_trial')))
    def test_right_turn(self):
        r=dict(R, kind='TURN',direction='RIGHT',limit=.2,
               proposed_linear_m_s=0.0,proposed_angular_rad_s=-.2)
        w=dict(W,kind='TURN')
        self.assertTrue(check(w=w,r=r))
