import copy
import math
import unittest
from pulse1_v6_single_segment_policy import (
    first_request, stop_request, start_ok, progress_ok, reached, step_count,
    should_emit_nominal_hold,
)
from pulse1_v6_supervisor_policy import health
from pulse1_v6_safety_contract import valid_request

WORLD={'scenario':'s1','navigation_mode':'AVOID','world':'empty',
       'obstacles':[{'name':'pulse1_s1_obstacle','x':5.,'y':0.,'size_x':1.,'size_y':1.4}]}

def inputs():
    return dict(world_paused=True, world_age=.05, cmd=(0.,0.), cmd_age=.04,
        guard={'source':'motor_guard_live','v6_armed':True,'guard_state':'BLOCKED',
               'motor_commands':True,'cmd_vel_exclusive':True,'emergency_latched':False,
               'nominal_stop_latched':False,'permitted_linear_m_s':0.,
               'permitted_angular_rad_s':0.},guard_age=.05,
        watchdog={'source':'v6_geofence_watchdog','armed':True,'action':'WAIT',
                  'ready_for_motion':False},watchdog_age=.05,
        v5_nodes=False,publisher_count=1,request_publishers=1,
        service_ready=True,supervisor={'source':'v6_supervisor','armed':True,
                                       'state':'HEALTHY'},supervisor_age=.05)

class V6First(unittest.TestCase):
    def test_drive_request_valid(self):
        r=first_request(WORLD,(0.,0.,0.))
        self.assertTrue(valid_request(r))
        self.assertEqual(r['limit'],.10)
        self.assertEqual(r['target'][1],0.)
    def test_stop_request_valid(self):
        r=stop_request(first_request(WORLD,(0.,0.,0.)))
        self.assertTrue(valid_request(r,moving=False))
        self.assertEqual(r['proposed_linear_m_s'],0)
    def test_reject_wrong_world(self):
        d=copy.deepcopy(WORLD);d['obstacles'][0]['x']=4.9
        with self.assertRaises(ValueError):first_request(d,(0.,0.,0.))
    def test_reject_wrong_initial_pose(self):
        for pos in ((.3,0.,0.),(0.,.1,0.),(0.,0.,.3),(math.nan,0,0)):
            with self.assertRaises(ValueError):first_request(WORLD,pos)
    def test_start_safe(self):self.assertTrue(start_ok(**inputs()))
    def test_start_missing_or_fake_guard(self):
        for bad in (None,{}, {'source':'motor_guard_live'}):
            d=inputs();d['guard']=bad
            self.assertFalse(start_ok(**d))
    def test_start_requires_v6_supervisor(self):
        for bad in (None,{}, {'source':'v6_supervisor','armed':False,'state':'HEALTHY'}):
            d=inputs();d['supervisor']=bad
            self.assertFalse(start_ok(**d))
    def test_start_requires_zero(self):
        d=inputs();d['cmd']=(.01,0.)
        self.assertFalse(start_ok(**d))
    def test_start_requires_exclusive_guard(self):
        d=inputs();d['guard']['cmd_vel_exclusive']=False
        self.assertFalse(start_ok(**d))
        d=inputs();d['publisher_count']=2
        self.assertFalse(start_ok(**d))
    def test_start_rejects_v5_nodes(self):
        d=inputs();d['v5_nodes']=True
        self.assertFalse(start_ok(**d))
    def test_start_requires_armed(self):
        for field in ('guard','watchdog'):
            d=inputs();d[field]['v6_armed' if field=='guard' else 'armed']=False
            self.assertFalse(start_ok(**d))
    def test_start_rejects_stale(self):
        for k in ('world_age','cmd_age','guard_age','watchdog_age','supervisor_age'):
            d=inputs();d[k]=10.
            self.assertFalse(start_ok(**d))
    def test_progress_bound(self):
        self.assertTrue(progress_ok((.099,0.,0.),(0.,0.,0.)))
        for pos in ((.20,0,0),(.05,.04,0),(.05,0,.1),None):
            self.assertFalse(progress_ok(pos,(0.,0.,0.)))
    def test_progress_reached(self):
        self.assertFalse(reached((.08,0.,0.),(0.,0.,0.)))
        self.assertTrue(reached((.093,0.,0.),(0.,0.,0.)))
    def test_step_budget(self):
        self.assertEqual(step_count((0.,0.,0.),(0.,0.,0.),.001),50)
        self.assertLessEqual(step_count((.113,0.,0.),(0.,0.,0.),.001),13)
        with self.assertRaises(ValueError):step_count((.1,0.,0.),(0.,0.,0.),.1)
    def test_supervisor_healthy(self):
        d=inputs()['guard'];d['mode']='AVOID'
        self.assertEqual(health(d,.1),'HEALTHY')
    def test_supervisor_stale(self):
        d=inputs()['guard'];d['mode']='AVOID'
        self.assertEqual(health(d,.9),'GUARD_LOST')
        self.assertEqual(health(d,math.nan),'GUARD_LOST')
    def test_supervisor_abort(self):
        d=inputs()['guard'];d['mode']='AVOID';d['emergency_latched']=True
        self.assertEqual(health(d,.1),'GUARD_UNSAFE')
    def test_missing_watchdog(self):
        d=inputs();d['watchdog']=None
        self.assertFalse(start_ok(**d))
    def test_guard_latch_rejected(self):
        d=inputs();d['guard']['nominal_stop_latched']=True
        self.assertFalse(start_ok(**d))
    def test_nominal_hold_requires_watchdog_and_reached(self):
        req=first_request(WORLD,(0.,0.,0.))
        wd=dict(source='v6_geofence_watchdog',action='STOPPING',
                reason='TARGET_REACHED',segment_id=req['segment_id'],
                waypoint_index=0,kind='ADVANCE')
        self.assertTrue(should_emit_nominal_hold(wd,req,(.094,0.,0.)))
        self.assertFalse(should_emit_nominal_hold(wd,req,(.07,0.,0.)))
        wd['segment_id']='wrong-segment'
        self.assertFalse(should_emit_nominal_hold(wd,req,(.094,0.,0.)))
        self.assertFalse(should_emit_nominal_hold(wd,None,(.094,0.,0.)))
    def test_control_discovery_required(self):
        d=inputs();d['service_ready']=False
        self.assertFalse(start_ok(**d))
    def test_request_topology_required(self):
        d=inputs();d['request_publishers']=2
        self.assertFalse(start_ok(**d))

if __name__=='__main__':unittest.main()
