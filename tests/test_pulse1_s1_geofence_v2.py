import unittest
from pulse1_s1_geofence_logic_v2 import GeofenceV2


def payload(x=0.1, moving=True, active=True):
    return dict(active=active, pose=(x, 0., 0.), pose_age_s=0.1,
                guard={"source":"motor_guard_live", "mode":"AVOID",
                       "motor_commands":True, "guard_state":"AUTHORIZED" if moving else "BLOCKED",
                       "permitted_linear_m_s":0.1 if moving else 0., "permitted_angular_rad_s":0.},
                guard_age_s=0.1, cmd=(0.1 if moving else 0., 0.),
                cmd_age_s=0.1, elapsed_s=2.)


class GeofenceV2Tests(unittest.TestCase):
    def test_nominal_stop(self):
        g = GeofenceV2()
        self.assertEqual(g.step(**payload())["action"], "MONITOR")
        stop = payload(x=0.205, active=False)
        self.assertEqual(g.step(**stop, completion_requested=True, now_s=10)["action"], "STOPPING")
        self.assertEqual(g.step(**stop, completion_requested=True, now_s=10.2)["action"], "STOPPING")
        off = payload(x=0.205, moving=False, active=False)
        self.assertEqual(g.step(**off, completion_requested=True, now_s=10.3,
                                world_paused=True, world_age_s=0.1)["action"], "COMPLETE")
        self.assertEqual(g.step(**off, now_s=20,
                                world_paused=True, world_age_s=0.1)["action"], "COMPLETE")

    def test_unexpected_deactivation(self):
        g = GeofenceV2()
        g.step(**payload())
        self.assertEqual(g.step(**payload(active=False))["reason"], "TRIAL_DEACTIVATED")
        self.assertEqual(g.step(**payload())["action"], "PAUSE")

    def test_early_done(self):
        g = GeofenceV2()
        g.step(**payload())
        self.assertEqual(g.step(**payload(x=0.0035, active=False),
                                completion_requested=True, now_s=10)["reason"], "EARLY_COMPLETION")

    def test_limit_still_emergency(self):
        g = GeofenceV2()
        g.step(**payload())
        self.assertEqual(g.step(**payload(x=0.22, active=False),
                                completion_requested=True, now_s=10)["reason"], "POSITION_LIMIT")

    def test_done_without_trial(self):
        g = GeofenceV2()
        self.assertEqual(g.step(**payload(x=0.205, active=False),
                                completion_requested=True, now_s=10)["reason"], "UNEXPECTED_COMPLETION")

    def test_stop_lost(self):
        g = GeofenceV2()
        g.step(**payload())
        g.step(**payload(x=0.205, active=False), completion_requested=True, now_s=10)
        self.assertEqual(g.step(**payload(x=0.205, active=False), now_s=10.2)["reason"], "STOP_REQUEST_LOST")

    def test_stop_timeout(self):
        g = GeofenceV2()
        g.step(**payload())
        g.step(**payload(x=0.205, active=False), completion_requested=True, now_s=10)
        self.assertEqual(g.step(**payload(x=0.205, active=False), completion_requested=True,
                                now_s=12.1)["reason"], "STOP_TIMEOUT")

    def test_no_restart(self):
        g = GeofenceV2()
        g.step(**payload())
        g.step(**payload(x=0.205, active=False), completion_requested=True, now_s=10)
        g.step(**payload(x=0.205, moving=False, active=False), completion_requested=True,
               now_s=10.2, world_paused=True, world_age_s=0.1)
        self.assertEqual(g.step(**payload(), world_paused=True,
                                world_age_s=0.1)["reason"], "UNSAFE_AFTER_COMPLETE")


if __name__ == '__main__': unittest.main()
