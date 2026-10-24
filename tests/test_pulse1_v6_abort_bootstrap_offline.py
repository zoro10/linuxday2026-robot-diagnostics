#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'scripts'))
from pulse1_v6_abort_resume_policy import *

G={'guard_state':'BLOCKED','emergency_latched':False,'nominal_stop_latched':False,
   'v6_armed':True,'cmd_vel_exclusive':True,'segment_id':None,
   'expected_next_segment_id':ABORT_ID}
F={'action':'PAUSE','reason':ABORT_REASON,'segment_id':None,'armed':True,'ready_for_motion':False}
S={'state':'HEALTHY'}
P=WP1_GAZEBO
class IncidentTest(unittest.TestCase):
 def audit(self): return capture_abort(G,F,S,True,(0.,0.),P,now=1000.)
 def fresh(self):
    g={**G,'expected_next_segment_id':None}
    f={**F,'action':'WAIT','reason':'NOT_STARTED','rearm_next_segment_id':None}
    return g,f
 def test_audit_valid(self): self.assertEqual(self.audit()['record_type'],'V6_ABORT_NOT_NOMINAL')
 def test_abort_not_complete(self):
    with self.assertRaises(ValueError): capture_abort(G,{**F,'action':'COMPLETE'},S,True,(0.,0.),P,now=1000.)
 def test_guard_nominal_false_required(self):
    with self.assertRaises(ValueError): capture_abort({**G,'nominal_stop_latched':True},F,S,True,(0.,0.),P,now=1000.)
 def test_abort_wrong_id(self):
    with self.assertRaises(ValueError): capture_abort({**G,'expected_next_segment_id':'other'},F,S,True,(0.,0.),P,now=1000.)
 def test_wrong_pose_capture(self):
    with self.assertRaises(ValueError): capture_abort(G,F,S,True,(0.,0.),(3.,2.3,1.5),now=1000.)
 def test_clean_restart(self):
    g,f=self.fresh()
    self.assertTrue(verify_bootstrap(self.audit(),g,f,S,True,(0.,0.),P,now=1001.))
 def test_still_aborted_rejected(self):
    with self.assertRaises(ValueError): verify_bootstrap(self.audit(),G,F,S,True,(0.,0.),P,now=1001.)
 def test_reset_expected_nonempty_rejected(self):
    g,f=self.fresh();g['expected_next_segment_id']='another'
    with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,S,True,(0.,0.),P,now=1001.)
 def test_physical_shift_rejected(self):
    g,f=self.fresh()
    with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,S,True,(0.,0.),(P[0]+.02,P[1],P[2]),now=1001.)
 def test_audit_stale(self):
    g,f=self.fresh()
    with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,S,True,(0.,0.),P,now=25000.)
 def test_no_motor_or_unpaused(self):
    g,f=self.fresh()
    with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,S,False,(0.,0.),P,now=1001.)
    with self.assertRaises(ValueError): verify_bootstrap(self.audit(),g,f,S,True,(0.1,0.),P,now=1001.)
 def test_emergency_rejected(self):
    g,f=self.fresh()
    with self.assertRaises(ValueError): verify_bootstrap(self.audit(),{**g,'emergency_latched':True},f,S,True,(0.,0.),P,now=1001.)
if __name__=='__main__':unittest.main(verbosity=2)
