import math
import unittest

from pulse1_s1_watchdog_contract import watchdog_allows_trial


def ready():
    return {
        "source": "s1_geofence_watchdog",
        "mode": "AVOID",
        "armed": True,
        "ready_for_motion": True,
        "trial_active": True,
        "action": "MONITOR",
        "reason": "WITHIN_LIMITS",
        "motor_commands": False,
    }


class WatchdogContractTests(unittest.TestCase):

    def test_valid_status(self):
        self.assertTrue(watchdog_allows_trial(ready(), 0.1))

    def test_missing_watchdog(self):
        self.assertFalse(watchdog_allows_trial(None, None))

    def test_observe_mode_denied(self):
        data = ready()
        data["armed"] = False
        data["ready_for_motion"] = False
        self.assertFalse(watchdog_allows_trial(data, 0.1))

    def test_stale_heartbeat(self):
        for age in (0.36, 1.0, None, math.nan):
            with self.subTest(age=age):
                self.assertFalse(
                    watchdog_allows_trial(ready(), age)
                )

    def test_alarm_denied(self):
        data = ready()
        data["action"] = "PAUSE"
        data["reason"] = "POSITION_LIMIT"
        self.assertFalse(watchdog_allows_trial(data, 0.1))

    def test_not_ready_denied(self):
        data = ready()
        data["ready_for_motion"] = False
        self.assertFalse(watchdog_allows_trial(data, 0.1))

    def test_trial_not_active(self):
        data = ready()
        data["trial_active"] = False
        self.assertFalse(watchdog_allows_trial(data, 0.1))

    def test_invalid_source(self):
        data = ready()
        data["source"] = "unknown"
        self.assertFalse(watchdog_allows_trial(data, 0.1))


if __name__ == "__main__":
    unittest.main()
