"""Test dei metodi ROS V6 senza avviare ROS, Gazebo o motori."""
import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pulse1_v6_ros_safety as ros
from pulse1_v6_safety_contract import V6GeofenceSegment

OLD = "v6-first-offline-001"
NEW = "v6-second-offline-002"


def make_guard():
    g = ros.V6MotorGuard.__new__(ros.V6MotorGuard)
    g.armed = True
    g.motor = object()
    g.nominal_stop_latched = True
    g.emergency_latched = False
    g.last_active = False
    g.active_segment_id = OLD
    g.expected_next_segment_id = None
    g.rearm_prepared = None
    g.pending_rearm_prepare = {
        "previous_segment_id": OLD,
        "next_segment_id": NEW,
        "operator_authorized": True,
    }
    g.pause_ack = True
    g.pause_error = None
    g.pause_future = None
    g.pause_sent_at = None
    g.pause_client = SimpleNamespace(
        service_is_ready=lambda: True
    )
    g.values = {
        "watchdog": {
            "source": "v6_geofence_watchdog",
            "armed": True,
            "segment_id": OLD,
            "action": "COMPLETE",
            "reason": "TARGET_REACHED",
            "pause_ack": True,
        },
        "cmd": (0.0, 0.0),
        "paused": True,
    }
    g.times = {key: time.monotonic() for key in g.values}
    g.get_logger = lambda: Mock()
    g.get_publishers_info_by_topic = lambda topic: (
        [object()] if topic == ros.CMD_TOPIC else []
    )
    return g


def make_geofence():
    gf = ros.V6Geofence.__new__(ros.V6Geofence)
    gf.core = V6GeofenceSegment(strict_v6_guard=True)
    gf.core.phase = "COMPLETE"
    gf.core.reason = "TARGET_REACHED"
    gf.core.request_id = OLD
    gf.pending_rearm = {
        "previous_segment_id": OLD,
        "next_segment_id": NEW,
        "operator_authorized": True,
    }
    gf.rearm_previous_id = None
    gf.pause_ack = True
    gf.pause_error = None
    gf.pause_future = None
    gf.pause_sent_at = None
    gf.values = {}
    gf.times = {}
    gf.get_logger = lambda: Mock()
    gf.get_publishers_info_by_topic = lambda topic: []
    return gf


class TestV6Handshake(unittest.TestCase):

    @patch.dict(os.environ, {"PULSE1_V6_REARM_ENABLE": "YES"})
    @patch.object(ros, "scenario_s1_avoid", return_value=True)
    def test_prepare_then_geofence_then_commit(self, _scenario):
        guard = make_guard()

        guard.process_guard_rearm_protocol(time.monotonic())

        self.assertIsNotNone(guard.rearm_prepared)
        self.assertTrue(guard.nominal_stop_latched)

        # PREPARE da solo non deve liberare il Guard.
        guard.process_guard_rearm_protocol(time.monotonic())
        self.assertTrue(guard.nominal_stop_latched)

        geofence = make_geofence()
        status = guard.guard_stop_snapshot()
        status["rearm_prepared_for"] = NEW
        status["rearm_previous_segment_id"] = OLD

        kw = dict(
            request=None,
            request_age_s=None,
            guard=status,
            guard_age_s=0.01,
            cmd=(0.0, 0.0),
            cmd_age_s=0.01,
            world_paused=True,
            world_age_s=0.01,
        )

        self.assertTrue(geofence.process_rearm(kw))
        self.assertEqual(geofence.core.phase, "WAIT")
        self.assertEqual(
            geofence.core.expected_next_segment_id, NEW
        )
        self.assertFalse(geofence.pause_ack)

        guard.values["watchdog"] = {
            "source": "v6_geofence_watchdog",
            "armed": True,
            "action": "WAIT",
            "reason": "REARM_VERIFIED",
            "ready_for_motion": False,
            "pause_error": None,
            "rearm_previous_segment_id": OLD,
            "rearm_next_segment_id": NEW,
        }
        guard.times["watchdog"] = time.monotonic()

        guard.process_guard_rearm_protocol(time.monotonic())

        self.assertFalse(guard.nominal_stop_latched)
        self.assertEqual(guard.expected_next_segment_id, NEW)
        self.assertFalse(guard.pause_ack)
        self.assertIsNone(guard.rearm_prepared)

    @patch.dict(os.environ, {"PULSE1_V6_REARM_ENABLE": "YES"})
    @patch.object(ros, "scenario_s1_avoid", return_value=True)
    def test_emergency_during_prepare_preserves_block(self, _scenario):
        guard = make_guard()
        guard.process_guard_rearm_protocol(time.monotonic())
        self.assertIsNotNone(guard.rearm_prepared)

        guard.emergency_latched = True
        guard.process_guard_rearm_protocol(time.monotonic())

        self.assertTrue(guard.nominal_stop_latched)
        self.assertTrue(guard.emergency_latched)
        self.assertIsNone(guard.rearm_prepared)


if __name__ == "__main__":
    unittest.main()
