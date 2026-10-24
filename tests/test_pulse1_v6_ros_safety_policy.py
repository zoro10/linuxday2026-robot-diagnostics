import copy
import unittest
from pulse1_v6_ros_safety_policy import watchdog_lease, motor_guard_status


def request():
    return {'source':'v6_segment_executor','mode':'AVOID','motor_commands':False,
            'phase':'DRIVE','segment_id':'segment-0001','waypoint_index':0,
            'kind':'ADVANCE','start_pose':[0.,0.,0.],'target':[2.36,0.],
            'limit':.2,'direction':'','proposed_linear_m_s':.1,
            'proposed_angular_rad_s':0.,
            'odom_ok':True,'lidar_ok':True,'rear_lidar_ok':True,'imu_ok':True}


def observer():
    return {'source':'avoid_observer','mode':'AVOID','state':'SENSOR_READY',
            'motor_commands':False, 'odom_ok':True,'lidar_ok':True,
            'rear_lidar_ok':True,'imu_ok':True}


def guard():
    return {'source':'motor_guard_live','mode':'AVOID','motor_commands':True,
            'v6_armed':True,'guard_state':'BLOCKED',
            'cmd_vel_exclusive':True,'emergency_latched':False}


def result():
    return {'action':'MONITOR','reason':'WITHIN_SEGMENT_ENVELOPE',
            'candidate_for_motion':True,'segment_id':'segment-0001',
            'kind':'ADVANCE','waypoint_index':0}


def opts():
    return dict(result=result(),arm_enabled=True,request=request(),request_age_s=.02,
                observer=observer(),observer_age_s=.02,pose=[0,0,0],pose_age_s=.02,
                guard=guard(),guard_age_s=.02,cmd_age_s=.02,world_age_s=.02,
                pause_service_ready=True)


class TestLease(unittest.TestCase):
    def test_startup_disarmed_no_request(self):
        d=opts(); d['arm_enabled']=False; d['request']=None
        d['result']={'action':'WAIT','reason':'NOT_STARTED',
                     'candidate_for_motion':False}
        self.assertIs(watchdog_lease(**d), False)
    def test_startup_armed_no_request(self):
        d=opts(); d['request']=None
        d['result']={'action':'WAIT','reason':'NOT_STARTED',
                     'candidate_for_motion':False}
        self.assertIs(watchdog_lease(**d), False)
    def test_malformed_request_types_fail_closed(self):
        for bad in (None, [], 0, '', {'source':'v6_segment_executor'}, {}):
            with self.subTest(request=repr(bad)):
                d=opts(); d['request']=bad
                self.assertIs(watchdog_lease(**d), False)
    def test_nominal(self): self.assertTrue(watchdog_lease(**opts()))
    def test_unarmed(self):
        d=opts();d['arm_enabled']=False;self.assertFalse(watchdog_lease(**d))
    def test_invalid_request(self):
        d=opts();d['request']['phase']='HOLD';self.assertFalse(watchdog_lease(**d))
    def test_other_segment(self):
        d=opts();d['result']['segment_id']='segment-2222';self.assertFalse(watchdog_lease(**d))
    def test_old_guard(self):
        d=opts();d['guard']['v6_armed']=False;self.assertFalse(watchdog_lease(**d))
    def test_missing_guard(self):
        d=opts();d['guard']=None;self.assertFalse(watchdog_lease(**d))
    def test_stale_guard(self):
        d=opts();d['guard_age_s']=.4;self.assertFalse(watchdog_lease(**d))
    def test_stale_world(self):
        d=opts();d['world_age_s']=1.;self.assertFalse(watchdog_lease(**d))
    def test_stale_odom(self):
        d=opts();d['pose_age_s']=.81;self.assertFalse(watchdog_lease(**d))
    def test_bad_odom(self):
        d=opts();d['pose']=[0,0];self.assertFalse(watchdog_lease(**d))
    def test_stale_observer(self):
        d=opts();d['observer_age_s']=.61;self.assertFalse(watchdog_lease(**d))
    def test_sensor_missing(self):
        d=opts();d['observer']['imu_ok']=False;self.assertFalse(watchdog_lease(**d))
    def test_service_down(self):
        d=opts();d['pause_service_ready']=False;self.assertFalse(watchdog_lease(**d))
    def test_cmd_stale(self):
        d=opts();d['cmd_age_s']=.4;self.assertFalse(watchdog_lease(**d))
    def test_pause_state(self):
        d=opts();d['result']['action']='PAUSE';self.assertFalse(watchdog_lease(**d))
    def test_guard_active(self):
        d=opts();d['guard']['guard_state']='AUTHORIZED';self.assertTrue(watchdog_lease(**d))
    def test_motor_status_disarmed(self):
        s=motor_guard_status(armed=False,velocity=(.1,0),request=request(),
                             pause_ready=True,exclusive=True)
        self.assertEqual(s['permitted_linear_m_s'],0.)
        self.assertFalse(s['motor_commands'])
    def test_motor_status_no_exclusivity(self):
        s=motor_guard_status(armed=True,velocity=(.1,0),request=request(),
                             pause_ready=True,exclusive=False)
        self.assertEqual(s['permitted_linear_m_s'],0.)
    def test_motor_status_active(self):
        s=motor_guard_status(armed=True,velocity=(0.,.2),request=request(),
                             pause_ready=True,exclusive=True)
        self.assertEqual(s['permitted_angular_rad_s'],.2)
        self.assertEqual(s['guard_state'],'AUTHORIZED')

if __name__ == '__main__':
    unittest.main()
