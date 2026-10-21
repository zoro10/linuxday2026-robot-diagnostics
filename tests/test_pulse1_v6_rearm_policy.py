import unittest
from pulse1_v6_rearm_policy import verify_rearm

OLD = "v6-segment-old-001"
NEW = "v6-segment-new-002"


def evidence():
    return dict(
        previous_id=OLD,
        next_id=NEW,
        operator_authorized=True,
        previous_executor_exited=True,
        watchdog={
            "source": "v6_geofence_watchdog",
            "armed": True,
            "segment_id": OLD,
            "action": "COMPLETE",
            "reason": "TARGET_REACHED",
            "pause_ack": True,
        },
        watchdog_age_s=0.01,
        guard={
            "source": "motor_guard_live",
            "mode": "AVOID",
            "motor_commands": True,
            "guard_state": "BLOCKED",
            "v6_armed": True,
            "cmd_vel_exclusive": True,
            "emergency_latched": False,
            "nominal_stop_latched": True,
            "segment_id": OLD,
            "permitted_linear_m_s": 0.0,
            "permitted_angular_rad_s": 0.0,
        },
        guard_age_s=0.01,
        cmd=(0.0, 0.0),
        cmd_age_s=0.01,
        world_paused=True,
        world_age_s=0.01,
    )


class TestRearmPolicy(unittest.TestCase):
    def test_valid_proof(self):
        self.assertTrue(verify_rearm(**evidence()))

    def test_operator_permission_required(self):
        d = evidence()
        d["operator_authorized"] = False
        self.assertFalse(verify_rearm(**d))

    def test_executor_exit_required(self):
        d = evidence()
        d["previous_executor_exited"] = False
        self.assertFalse(verify_rearm(**d))

    def test_emergency_cannot_rearm(self):
        d = evidence()
        d["guard"]["emergency_latched"] = True
        self.assertFalse(verify_rearm(**d))

    def test_wrong_segment_cannot_rearm(self):
        d = evidence()
        d["watchdog"]["segment_id"] = NEW
        self.assertFalse(verify_rearm(**d))

    def test_world_must_be_paused(self):
        d = evidence()
        d["world_paused"] = False
        self.assertFalse(verify_rearm(**d))

    def test_nonzero_motor_command_denied(self):
        d = evidence()
        d["cmd"] = (0.1, 0.0)
        self.assertFalse(verify_rearm(**d))

    def test_stale_watchdog_denied(self):
        d = evidence()
        d["watchdog_age_s"] = 1.0
        self.assertFalse(verify_rearm(**d))


if __name__ == "__main__":
    unittest.main()
