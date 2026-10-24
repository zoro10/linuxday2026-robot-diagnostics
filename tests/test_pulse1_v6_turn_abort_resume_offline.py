#!/usr/bin/env python3
import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'scripts'))
from pulse1_v6_abort_resume_policy import *

G={'guard_state':'ABORT','emergency_latched':True,'nominal_stop_latched':False,
   'v6_armed':True,'cmd_vel_exclusive':True, 'expected_next_segment_id':TURN_ABORT_ID}
F={'action':'PAUSE','reason':TURN_ABORT_REASON,'segment_id':TURN_ABORT_ID,
   'armed':True,'ready_for_motion':False}
S={'state':'EMERGENCY_PAUSE','pause_ack':True,'motor_commands':False}
P=TURN_ABORT_GAZEBO
class TurnIncidentAuditTest(unittest.TestCase):
    def audit(self): return capture_turn_abort(G,F,S,True,(0.,0.),P,now=1000.)
    def clean(self):
        g={**G,'guard_state':'BLOCKED','emergency_latched':False,
           'expected_next_segment_id':None}
        f={**F,'action':'WAIT','reason':'NOT_STARTED','segment_id':None,
           'rearm_next_segment_id':None}
        sup={'state':'HEALTHY'}
        return g,f,sup
    def test_partial_motion_explicit_and_not_nominal(self):
        a=self.audit(); self.assertTrue(a['motion_authorized']);
        self.assertTrue(a['partial_motion_observed']); self.assertFalse(a['nominal_stop'])
    def test_wrong_abort_id_rejected(self):
        with self.assertRaises(ValueError): capture_turn_abort({**G,'expected_next_segment_id':'other'},F,S,True,(0,0),P,now=1000)
    def test_wrong_geofence_reason_rejected(self):
        with self.assertRaises(ValueError): capture_turn_abort(G,{**F,'reason':'OBSERVER_STALE'},S,True,(0,0),P,now=1000)
    def test_supervisor_must_have_acknowledged_pause(self):
        with self.assertRaises(ValueError): capture_turn_abort(G,F,{**S,'pause_ack':False},True,(0,0),P,now=1000)
    def test_motor_must_be_zero_and_world_paused(self):
        for paused,cmd in ((False,(0,0)),(True,(.01,0))):
            with self.assertRaises(ValueError): capture_turn_abort(G,F,S,paused,cmd,P,now=1000)
    def test_wrong_position_rejected(self):
        with self.assertRaises(ValueError): capture_turn_abort(G,F,S,True,(0,0),(P[0]+.04,P[1],P[2]),now=1000)
    def test_bootstrap_accepts_clean_restart(self):
        g,f,s=self.clean()
        self.assertTrue(verify_bootstrap(self.audit(),g,f,s,True,(0.,0.),P,now=1001))
    def test_bootstrap_rejects_still_aborted(self):
        with self.assertRaises(ValueError): verify_bootstrap(self.audit(),G,F,S,True,(0.,0.),P,now=1001)
    def test_bootstrap_rejects_pose_changed(self):
        g,f,s=self.clean()
        with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,s,True,(0.,0.),(P[0]+.025,P[1],P[2]),now=1001)
    def test_bootstrap_rejects_audit_falsely_nominal(self):
        g,f,s=self.clean();a=self.audit();a['motion_authorized']=False
        with self.assertRaises(ValueError): verify_bootstrap(a,g,f,s,True,(0.,0.),P,now=1001)
    def test_bootstrap_rejects_old_id_stale(self):
        g,f,s=self.clean()
        with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,s,True,(0.,0.),P,now=25001)
    def test_bootstrap_rejects_pending_segment(self):
        g,f,s=self.clean();g['expected_next_segment_id']=TURN_ABORT_ID
        with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,s,True,(0.,0.),P,now=1001)
if __name__=='__main__':unittest.main(verbosity=2)
