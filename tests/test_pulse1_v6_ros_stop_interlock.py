"""Regression for V6's real controller handover: V5 must never satisfy V6 STOP."""
import unittest
from pulse1_v6_safety_contract import V6GeofenceSegment, stopped_independently
from pulse1_v6_ros_safety_policy import watchdog_lease, motor_guard_status
from test_pulse1_v6_safety_contract import request, observer, samples, blocked_guard
from test_pulse1_v6_ros_safety_policy import opts


def guard_v6(*, nominal=False, segment_id=None, exclusive=True, emergency=False):
    return dict(source='motor_guard_live', mode='AVOID', motor_commands=True,
                guard_state='BLOCKED', v6_armed=True,
                cmd_vel_exclusive=exclusive, emergency_latched=emergency,
                nominal_stop_latched=nominal, segment_id=segment_id,
                permitted_linear_m_s=0., permitted_angular_rad_s=0.)


class TestV6ControllerHandover(unittest.TestCase):
    def test_v5_cannot_arm_strict_v6_geofence(self):
        core = V6GeofenceSegment(strict_v6_guard=True)
        status = core.tick(**samples())
        self.assertEqual((status['action'], status['reason']),
                         ('PAUSE', 'START_NOT_STOPPED'))

    def test_v6_zero_exclusive_can_enter_monitor(self):
        core = V6GeofenceSegment(strict_v6_guard=True)
        status = core.tick(**samples(guard=guard_v6()))
        self.assertEqual(status['action'], 'MONITOR')
        self.assertFalse(status['ready_for_motion'])  # pure core never leases

    def test_v6_guard_without_exclusivity_cannot_start(self):
        core = V6GeofenceSegment(strict_v6_guard=True)
        self.assertEqual(core.tick(**samples(guard=guard_v6(exclusive=False)))['action'], 'PAUSE')

    def test_emergency_guard_cannot_start(self):
        core = V6GeofenceSegment(strict_v6_guard=True)
        self.assertEqual(core.tick(**samples(guard=guard_v6(emergency=True)))['action'], 'PAUSE')

    def test_stopping_needs_same_segment_nominal_latch(self):
        core = V6GeofenceSegment(strict_v6_guard=True)
        self.assertEqual(core.tick(**samples(guard=guard_v6()))['action'], 'MONITOR')
        self.assertEqual(core.tick(**samples(now_s=101., pose=(.2,0.,0.),guard=guard_v6()))['action'], 'STOPPING')
        r=request();r.update(phase='HOLD',reason='SEGMENT_TARGET_REACHED',proposed_linear_m_s=0.)
        for guard in (guard_v6(), guard_v6(nominal=True,segment_id='segment-other'),
                      blocked_guard(), guard_v6(nominal=True,segment_id='segment-0001',exclusive=False)):
            st=core.tick(**samples(request=r,now_s=101.5,pose=(.2,0.,0.),guard=guard,pause_ack=True))
            self.assertEqual(st['action'], 'STOPPING')
        final=core.tick(**samples(request=r,now_s=101.6,pose=(.2,0.,0.),
                                  guard=guard_v6(nominal=True,segment_id='segment-0001'),pause_ack=True))
        self.assertEqual((final['action'],final['reason']),('COMPLETE','TARGET_REACHED'))

    def test_lease_needs_exclusive_guard(self):
        d=opts(); d['guard']['cmd_vel_exclusive']=False
        self.assertIs(watchdog_lease(**d), False)

    def test_lease_rejects_latched_emergency(self):
        d=opts(); d['guard']['emergency_latched']=True
        self.assertIs(watchdog_lease(**d), False)

    def test_guard_status_exports_exclusive(self):
        d=motor_guard_status(armed=True, velocity=(0.,0.), request=request(),
                              pause_ready=True, exclusive=True)
        self.assertIs(d['cmd_vel_exclusive'],True)
        d=motor_guard_status(armed=True, velocity=(0.,0.), request=request(),
                              pause_ready=True, exclusive=False)
        self.assertIs(d['cmd_vel_exclusive'],False)

if __name__ == '__main__':
    unittest.main()
