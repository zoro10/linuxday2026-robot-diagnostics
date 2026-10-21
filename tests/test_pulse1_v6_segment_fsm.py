import math
import unittest
from pulse1_v6_segment_fsm import V6SegmentGate, Segment

GUARD = dict(source='motor_guard_live', mode='AVOID', motor_commands=True,
             guard_state='BLOCKED', permitted_linear_m_s=0, permitted_angular_rad_s=0)
ACK = dict(pose_age_s=.01, world_step_ack=True)
STOP = dict(paused=True, paused_age_s=.01, guard=GUARD, guard_age_s=.01,
            cmd=(0.,0.), cmd_age_s=.01)

def shadow(kind='ADVANCE', limit=.2, direction='LEFT', pose=(0,0,0), index=0):
    c=dict(waypoint_index=index, start_pose=list(pose), target=[2.3685,0],
           requires_independent_stop_ack=True, kind=kind)
    if kind=='ADVANCE': c['max_distance_m']=limit
    if kind=='TURN': c['max_angle_rad']=limit; c['direction']=direction
    return dict(source='waypoint_segment_shadow', mode='AVOID', state='CANDIDATE',
                reason='OFFLINE_PROPOSAL_ONLY', candidate=c,
                motion_authorized=False, motor_commands=False,
                proposed_linear_m_s=0, proposed_angular_rad_s=0)

class TestV6(unittest.TestCase):
    def test_valid_prepare(self):
        g=V6SegmentGate(); self.assertEqual(g.prepare(shadow(),(0,0,0),**STOP),'PREPARED')
    def test_bad_shadow(self):
        g=V6SegmentGate(); s=shadow(); s['motion_authorized']=True
        self.assertEqual(g.prepare(s,(0,0,0),**STOP),'ABORT')
    def test_bad_guard(self):
        g=V6SegmentGate(); data=dict(STOP); data['guard']=dict(GUARD, guard_state='AUTHORIZED')
        self.assertEqual(g.prepare(shadow(),(0,0,0),**data),'ABORT')
    def test_bad_cmd(self):
        g=V6SegmentGate(); data=dict(STOP); data['cmd']=(.1,0)
        self.assertEqual(g.prepare(shadow(),(0,0,0),**data),'ABORT')
    def test_stale_paused(self):
        g=V6SegmentGate(); data=dict(STOP); data['paused_age_s']=.75
        self.assertEqual(g.prepare(shadow(),(0,0,0),**data),'ABORT')
    def test_invalid_pose(self):
        g=V6SegmentGate(); self.assertEqual(g.prepare(shadow(),(float('nan'),0,0),**STOP),'ABORT')
    def test_start_disagree(self):
        g=V6SegmentGate(); self.assertEqual(g.prepare(shadow(),(.05,0,0),**STOP),'ABORT')
    def test_distance_over_cap(self):
        with self.assertRaises(ValueError): Segment.from_shadow(shadow(limit=.25))
    def test_turn_over_cap(self):
        with self.assertRaises(ValueError): Segment.from_shadow(shadow('TURN',.3))
    def test_ack_required(self):
        with self.assertRaises(ValueError):
            s=shadow();s['candidate']['requires_independent_stop_ack']=False
            Segment.from_shadow(s)
    def test_no_motion_without_dual_authorization(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        self.assertEqual(g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,
                                                     v6_motor_authorized=False),'ABORT')
    def test_forward_checkpoint(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        self.assertEqual(g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,
                                                     v6_motor_authorized=True),'ACTIVE')
        self.assertEqual(g.progress((.12,0,0),now_s=1.4,pose_age_s=.01),'ACTIVE')
        self.assertEqual(g.progress((.199,0,0),now_s=2,pose_age_s=.01),'STOP_REQUESTED')
        self.assertEqual(g.acknowledge_stop((.202,0,0),now_s=2.2,independent_stop_ack=True,**ACK,**STOP),'STOP_CONFIRMED')
        self.assertEqual(g.next_segment(),'IDLE')
        self.assertEqual(g.ack_count,1)
    def test_overshoot_latches_abort(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        self.assertEqual(g.progress((.23,0,0),now_s=2,pose_age_s=.01),'ABORT')
    def test_lateral_deviation(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        self.assertEqual(g.progress((.1,.05,0),now_s=2,pose_age_s=.01),'ABORT')
    def test_early_stop_rejected(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        g.progress((.199,0,0),now_s=2,pose_age_s=.01)
        self.assertEqual(g.acknowledge_stop((.17,0,0),now_s=2.2,independent_stop_ack=True,**ACK,**STOP),'ABORT')
    def test_no_stop_ack(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        g.progress((.2,0,0),now_s=2,pose_age_s=.01)
        self.assertEqual(g.acknowledge_stop((.2,0,0),now_s=2.1,independent_stop_ack=False,**ACK,**STOP),'ABORT')
    def test_turn_left(self):
        g=V6SegmentGate();a=.20;g.prepare(shadow('TURN',a),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        self.assertEqual(g.progress((0,0,.198),now_s=2,pose_age_s=.01),'STOP_REQUESTED')
        self.assertEqual(g.acknowledge_stop((0,0,.201),now_s=2.2,independent_stop_ack=True,**ACK,**STOP),'STOP_CONFIRMED')
    def test_turn_right(self):
        g=V6SegmentGate();g.prepare(shadow('TURN',.2,'RIGHT'),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        self.assertEqual(g.progress((0,0,-.2),now_s=2,pose_age_s=.01),'STOP_REQUESTED')
        self.assertEqual(g.acknowledge_stop((0,0,-.2),now_s=2.2,independent_stop_ack=True,**ACK,**STOP),'STOP_CONFIRMED')
    def test_turn_wrong_direction(self):
        g=V6SegmentGate();g.prepare(shadow('TURN',.2,'RIGHT'),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        self.assertEqual(g.progress((0,0,.1),now_s=2,pose_age_s=.01),'ABORT')
    def test_stopping_timeout(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        g.progress((.20,0,0),now_s=2,pose_age_s=.01)
        self.assertEqual(g.acknowledge_stop((.20,0,0),now_s=10,independent_stop_ack=True,**ACK,**STOP),'ABORT')
    def test_no_next_without_stop(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        self.assertEqual(g.next_segment(),'ABORT')

    def test_stale_pose_progress_aborts(self):
        g=V6SegmentGate(); g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        self.assertEqual(g.progress((.1,0,0),now_s=2,pose_age_s=1.5),'ABORT')
    def test_no_physics_step_ack_aborts(self):
        g=V6SegmentGate(); g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        g.progress((.2,0,0),now_s=2,pose_age_s=.01)
        ack=dict(ACK,world_step_ack=False)
        self.assertEqual(g.acknowledge_stop((.2,0,0),now_s=2.1,independent_stop_ack=True,**ack,**STOP),'ABORT')
    def test_no_automatic_replay(self):
        g=V6SegmentGate();g.prepare(shadow(),(0,0,0),**STOP)
        g.record_authorized_motion(now_s=1,v6_watchdog_confirmed=True,v6_motor_authorized=True)
        g.progress((.2,0,0),now_s=2,pose_age_s=.01)
        g.acknowledge_stop((.2,0,0),now_s=2.1,independent_stop_ack=True,**ACK,**STOP)
        g.next_segment()
        self.assertEqual(g.prepare(shadow(pose=(0,0,0)),(0,0,0),**STOP),'ABORT')
if __name__ == '__main__': unittest.main()
