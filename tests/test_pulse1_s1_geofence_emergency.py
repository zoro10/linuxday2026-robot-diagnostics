import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from pulse1_s1_geofence_logic import Geofence
from pulse1_s1_geofence_watchdog import GeofenceObserver


def trial_request():
    return {
        "source": "avoid_navigator",
        "mode": "AVOID",
        "state": "GO_TO_GOAL",
        "trial_phase": "DRIVE",
        "trial_reason": "LIMITED_FORWARD_TRIAL",
        "motor_commands": False,
        "proposed_linear_m_s": 0.10,
        "proposed_angular_rad_s": 0.0,
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
    }


def fake_node(armed):
    node = SimpleNamespace(
        armed=armed,
        values={"nav": trial_request()},
        started_at=None,
        geofence=Geofence(),
        last_pause_at=None,
        output=Mock(),
        get_logger=Mock(),
    )
    node.age = lambda name, now: 0.1
    return node


class EmergencyTests(unittest.TestCase):

    def test_armed_alarm_requests_pause(self):
        node = fake_node(True)

        with patch(
            "pulse1_s1_geofence_watchdog.pause_gazebo",
            return_value=True,
        ) as pause:
            GeofenceObserver.control(node)
            pause.assert_called_once_with()

        msg = node.output.publish.call_args[0][0]
        status = json.loads(msg.data)

        self.assertEqual(status["action"], "PAUSE")
        self.assertFalse(status["ready_for_motion"])

    def test_observer_never_pauses(self):
        node = fake_node(False)

        with patch(
            "pulse1_s1_geofence_watchdog.pause_gazebo"
        ) as pause:
            GeofenceObserver.control(node)
            pause.assert_not_called()

    def test_alarm_remains_latched(self):
        node = fake_node(True)

        with patch(
            "pulse1_s1_geofence_watchdog.pause_gazebo",
            return_value=True,
        ):
            GeofenceObserver.control(node)

        self.assertIsNotNone(node.geofence.alarm)


if __name__ == "__main__":
    unittest.main()


class PositionEmergencyTests(unittest.TestCase):

    def test_monitor_then_position_pause(self):
        node = fake_node(True)

        node.values.update({
            "pose": (0.0, 0.0, 0.0),
            "cmd": (0.0, 0.0),
            "guard": {
                "source": "motor_guard_live",
                "mode": "AVOID",
                "motor_commands": True,
                "guard_state": "BLOCKED",
                "permitted_linear_m_s": 0.0,
                "permitted_angular_rad_s": 0.0,
            },
        })

        with patch(
            "pulse1_s1_geofence_watchdog.pause_gazebo",
            return_value=True,
        ) as pause:

            GeofenceObserver.control(node)

            first = json.loads(
                node.output.publish.call_args.args[0].data
            )

            self.assertEqual(first["action"], "MONITOR")
            self.assertTrue(first["ready_for_motion"])
            pause.assert_not_called()

            node.values["pose"] = (0.23, 0.0, 0.0)

            GeofenceObserver.control(node)

            second = json.loads(
                node.output.publish.call_args.args[0].data
            )

            self.assertEqual(second["action"], "PAUSE")
            self.assertEqual(
                second["reason"], "POSITION_LIMIT"
            )
            self.assertFalse(second["ready_for_motion"])
            pause.assert_called_once_with()

            node.values["pose"] = (0.0, 0.0, 0.0)

            GeofenceObserver.control(node)

            third = json.loads(
                node.output.publish.call_args.args[0].data
            )

            self.assertEqual(third["action"], "PAUSE")
            self.assertEqual(
                third["reason"], "POSITION_LIMIT"
            )
