import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
START = ROOT / "scripts" / "start_demo.sh"


class SafeStartTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.script = START.read_text()

    def test_explicit_scenario_gate(self):
        self.assertIn(
            'PULSE1_AVOID_START_ENABLE:-',
            self.script,
        )

    def test_turn_navigator_starts_disarmed(self):
        start = self.script.index(
            'tmux new-window -t "$SESSION" -n turn_nav'
        )
        end = self.script.index(
            'echo "[4c/4] Avvio Motor Guard AVOID..."',
            start,
        )
        section = self.script[start:end]

        self.assertIn(
            "exec python3 scripts/pulse1_turn_navigator.py",
            section,
        )
        self.assertNotIn("--armed", section)
        self.assertNotIn("PULSE1_AVOID_TURN_ARM", section)

    def test_motor_guard_remains_separate(self):
        self.assertIn(
            "PULSE1_NAV_MODE=AVOID PULSE1_AVOID_MOTOR_ARM=YES",
            self.script,
        )


if __name__ == "__main__":
    unittest.main()
