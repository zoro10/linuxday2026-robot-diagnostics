import unittest
from pulse1_v6_safety_contract import V6GeofenceSegment

OLD = "v6-first-test-old"
NEW = "v6-next-test-new"

def stopped_guard():
    return {
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
    }

def completed():
    gate = V6GeofenceSegment(strict_v6_guard=True)
    gate.phase = "COMPLETE"
    gate.reason = "TARGET_REACHED"
    gate.request_id = OLD
    return gate

def proof():
    return {
        "previous_segment_id": OLD,
        "next_segment_id": NEW,
        "operator_authorized": True,
        "previous_executor_exited": True,
        "pause_ack": True,
        "guard": stopped_guard(),
        "guard_age_s": 0.01,
        "cmd": (0.0, 0.0),
        "cmd_age_s": 0.01,
        "world_paused": True,
        "world_age_s": 0.01,
    }

class TestGeofenceRearm(unittest.TestCase):
    def test_confirmed_stop_can_prepare_next_segment(self):
        gate = completed()
        self.assertTrue(gate.rearm_after_confirmed_stop(**proof()))
        self.assertEqual(gate.phase, "WAIT")
        self.assertEqual(gate.reason, "REARM_VERIFIED")

    def test_emergency_pause_cannot_rearm(self):
        gate = completed()
        gate.phase = "PAUSE"
        self.assertFalse(gate.rearm_after_confirmed_stop(**proof()))
        self.assertEqual(gate.phase, "PAUSE")

    def test_missing_stop_proof_cannot_rearm(self):
        gate = completed()
        args = proof()
        args["guard"]["nominal_stop_latched"] = False
        self.assertFalse(gate.rearm_after_confirmed_stop(**args))
        self.assertEqual(gate.phase, "COMPLETE")

if __name__ == "__main__":
    unittest.main()
