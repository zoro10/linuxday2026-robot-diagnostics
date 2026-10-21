import unittest
from pathlib import Path

from pulse1_v6_segment_fsm import Segment, V6SegmentGate

GUARD = {
    "source": "motor_guard_live", "mode": "AVOID",
    "motor_commands": True, "guard_state": "BLOCKED",
    "permitted_linear_m_s": 0.0, "permitted_angular_rad_s": 0.0,
}
STOP = {
    "paused": True, "paused_age_s": 0.01,
    "guard": GUARD, "guard_age_s": 0.01,
    "cmd": (0.0, 0.0), "cmd_age_s": 0.01,
}

def request():
    return {
        "source": "v6_segment_executor", "mode": "AVOID",
        "motor_commands": False, "phase": "DRIVE",
        "segment_id": "v6-first-test", "waypoint_index": 0,
        "kind": "ADVANCE", "direction": "",
        "start_pose": [0.0, 0.0, 0.0],
        "target": [2.3685, 0.0], "limit": 0.10,
        "proposed_linear_m_s": 0.10,
        "proposed_angular_rad_s": 0.0,
    }

class TestExecutorFSMAdapter(unittest.TestCase):
    def test_executor_request_parsed(self):
        seg = Segment.from_executor_request(request())
        self.assertEqual(seg.kind, "ADVANCE")
        self.assertAlmostEqual(seg.limit, 0.10)

    def test_wrong_source_rejected(self):
        r = request()
        r["source"] = "untrusted"
        with self.assertRaises(ValueError):
            Segment.from_executor_request(r)

    def test_full_prepare_progress_stop_lifecycle(self):
        gate = V6SegmentGate()
        self.assertEqual(
            gate.prepare_executor_request(request(), (0.0, 0.0, 0.0), **STOP),
            "PREPARED",
        )
        self.assertEqual(
            gate.record_authorized_motion(
                now_s=1.0,
                v6_watchdog_confirmed=True,
                v6_motor_authorized=True,
            ),
            "ACTIVE",
        )
        self.assertEqual(
            gate.progress(
                (0.09, 0.0, 0.0), now_s=1.1, pose_age_s=0.01,
                reached_tolerance_m=0.012,
            ),
            "STOP_REQUESTED",
        )
        self.assertEqual(
            gate.acknowledge_stop(
                (0.09, 0.0, 0.0), now_s=1.2, pose_age_s=0.01,
                world_step_ack=True, independent_stop_ack=True, **STOP,
            ),
            "STOP_CONFIRMED",
        )

    def test_replay_after_confirmed_stop_rejected(self):
        gate = V6SegmentGate()
        gate.prepare_executor_request(request(), (0.0, 0.0, 0.0), **STOP)
        gate.record_authorized_motion(
            now_s=1.0, v6_watchdog_confirmed=True, v6_motor_authorized=True
        )
        gate.progress(
            (0.09, 0.0, 0.0), now_s=1.1, pose_age_s=0.01,
            reached_tolerance_m=0.012,
        )
        gate.acknowledge_stop(
            (0.09, 0.0, 0.0), now_s=1.2, pose_age_s=0.01,
            world_step_ack=True, independent_stop_ack=True, **STOP,
        )
        self.assertEqual(gate.next_segment(), "IDLE")
        self.assertEqual(
            gate.prepare_executor_request(request(), (0.0, 0.0, 0.0), **STOP),
            "ABORT",
        )

    def test_executor_is_wired_to_gate(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "scripts" / "pulse1_v6_single_segment.py"
        ).read_text()
        for token in (
            "V6SegmentGate",
            "prepare_executor_request",
            "record_authorized_motion",
            "gate.progress(",
            "gate.acknowledge_stop(",
        ):
            self.assertIn(token, source)

    def test_terminal_progress_not_repeated_after_stop_requested(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "scripts" / "pulse1_v6_single_segment.py"
        ).read_text()
        self.assertEqual(source.count("if st=='TERMINAL':"), 1)
        block = source.split("if st=='TERMINAL':", 1)[1]
        block = block.split(
            "print('CONTROL_STEP interrupted by nominal PAUSE'", 1
        )[0]
        self.assertIn("if gate.phase == 'ACTIVE':", block)
        self.assertIn("elif gate.phase != 'STOP_REQUESTED':", block)

if __name__ == "__main__":
    unittest.main()
