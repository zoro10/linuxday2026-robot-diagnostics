"""Standalone regressions for a global five-attempt budget; no ROS/Gazebo."""
import io
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
from pulse1_v6_preflight_retry import (refresh_with_budget,
    check_replan_or_refresh, odom_snapshot, ensure_stationary, MAX_REFRESH_ATTEMPTS)


class FakeNode:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = 0
        self.position = (2.1, 2.8, 1.0)
        self.last = None
        self.safe = True

    def stopped(self):
        return self.safe

    def wait(self, pred, name):
        if not pred():
            raise RuntimeError("TIMEOUT_" + name)

    def refresh_sensors_before_rearm(self, kind):
        self.calls += 1
        self.last = self.outcomes[self.calls - 1]
        if self.last in ("nofresh", "pending", "obstacle"):
            names = {"nofresh":"SENSOR_PREFLIGHT_NO_FRESH_DATA",
                     "pending":"SENSOR_STEP_PENDING_OR_TIMEOUT",
                     "obstacle":"SENSOR_PREFLIGHT_DENIED:TURN_OBSTACLE"}
            raise RuntimeError(names[self.last])
        if self.last == "drift":
            self.position = (2.11, 2.8, 1.0)

    def sensor_gate(self, kind):
        return ((False,"OBSERVER_NOT_HEALTHY") if self.last == "stale"
                else (False,"TURN_CLEARANCE_DENIED") if self.last == "deny"
                else (True,"TURN_CLEAR_FULL_RAY_ENVELOPE"))

    def value(self, name):
        assert name == "observer"
        x,y,yaw = self.position
        return {"readings":{"odom":{"x":x,"y":y,"yaw_rad":yaw}}}


class RetryTests(unittest.TestCase):
    def stage(self, node, budget=MAX_REFRESH_ATTEMPTS):
        with redirect_stdout(io.StringIO()):
            return refresh_with_budget(node,"TURN",budget,phase="INITIAL")

    def post(self, node, used):
        with redirect_stdout(io.StringIO()):
            return check_replan_or_refresh(node,"TURN",(2.1,2.8,1.0),used)

    def test_initial_first_success(self):
        n=FakeNode(["ok"])
        self.assertEqual(self.stage(n)[1],1)
        self.assertEqual(n.calls,1)

    def test_initial_fifth_success(self):
        n=FakeNode(["stale"]*4+["ok"])
        self.assertEqual(self.stage(n)[1],5)
        self.assertEqual(n.calls,5)

    def test_initial_five_stale_stop(self):
        n=FakeNode(["stale"]*5)
        with self.assertRaisesRegex(RuntimeError,"EXHAUSTED:INITIAL:OBSERVER_NOT_HEALTHY"):
            self.stage(n)
        self.assertEqual(n.calls,5)

    def test_nofresh_then_success(self):
        n=FakeNode(["nofresh","ok"])
        self.assertEqual(self.stage(n)[1],2)
        self.assertEqual(n.calls,2)

    def test_nofresh_five_stop(self):
        n=FakeNode(["nofresh"]*5)
        with self.assertRaisesRegex(RuntimeError,"EXHAUSTED:INITIAL:NO_FRESH_DATA"):
            self.stage(n)
        self.assertEqual(n.calls,5)

    def test_pending_step_never_retried(self):
        n=FakeNode(["pending","ok"])
        with self.assertRaisesRegex(RuntimeError,"SENSOR_STEP_PENDING_OR_TIMEOUT"):
            self.stage(n)
        self.assertEqual(n.calls,1)

    def test_obstacle_never_retried(self):
        n=FakeNode(["obstacle","ok"])
        with self.assertRaisesRegex(RuntimeError,"SENSOR_PREFLIGHT_DENIED:TURN_OBSTACLE"):
            self.stage(n)
        self.assertEqual(n.calls,1)

    def test_gate_clearance_denial_never_retried(self):
        n=FakeNode(["deny","ok"])
        with self.assertRaisesRegex(RuntimeError,"SENSOR_PREFLIGHT_DENIED:TURN_CLEARANCE_DENIED"):
            self.stage(n)
        self.assertEqual(n.calls,1)

    def test_replan_already_fresh_no_second_refresh(self):
        n=FakeNode(["ok"])
        self.stage(n)
        self.assertEqual(self.post(n,1),1)
        self.assertEqual(n.calls,1)

    def test_post_gazebo_four_more_retries_only(self):
        n=FakeNode(["ok","stale","stale","stale","ok"])
        self.stage(n)
        n.last="stale"  # sensor aged during blocking Gazebo read
        self.assertEqual(self.post(n,1),5)
        self.assertEqual(n.calls,5)

    def test_post_gazebo_global_budget_exhausted(self):
        n=FakeNode(["ok","stale","stale","stale","stale"])
        self.stage(n)
        n.last="stale"
        with self.assertRaisesRegex(RuntimeError,"EXHAUSTED:REPLAN:OBSERVER_NOT_HEALTHY"):
            self.post(n,1)
        self.assertEqual(n.calls,5)

    def test_post_five_already_spent_no_refresh(self):
        n=FakeNode(["ok"])
        n.last="stale"
        with self.assertRaisesRegex(RuntimeError,"EXHAUSTED:REPLAN:BUDGET_5"):
            self.post(n,5)
        self.assertEqual(n.calls,0)

    def test_rover_drift_no_retry(self):
        n=FakeNode(["ok","drift","ok"])
        self.stage(n)
        n.last="stale"
        with self.assertRaisesRegex(RuntimeError,"SENSOR_REFRESH_ROVER_MOVED"):
            self.post(n,1)
        self.assertEqual(n.calls,2)

    def test_interlock_loss_no_retry(self):
        n=FakeNode(["ok"])
        n.safe=False
        with self.assertRaisesRegex(RuntimeError,"TIMEOUT_SENSOR_STOP_BEFORE_REFRESH"):
            self.stage(n)
        self.assertEqual(n.calls,0)

    def test_odom_missing_denied(self):
        class Missing:
            def value(self, key): return {"readings":{}}
        with self.assertRaisesRegex(RuntimeError,"SENSOR_ODOM_MISSING"):
            odom_snapshot(Missing())

    def test_angular_wrap_and_distance_limit(self):
        ensure_stationary((0,0,3.14),(0,0,-3.14))
        with self.assertRaisesRegex(RuntimeError,"SENSOR_REFRESH_ROVER_MOVED"):
            ensure_stationary((0,0,0),(0.006,0,0))

if __name__ == "__main__":
    unittest.main(verbosity=2)
