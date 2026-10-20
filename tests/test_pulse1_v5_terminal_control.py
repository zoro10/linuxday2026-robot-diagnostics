import time

import pytest

from pulse1_nominal_stop_v5 import terminal_step_stopped


ERROR = (
    "Timeout ControlWorld: target=5750, "
    "ACK=True, PAUSED_TARGET=False, "
    "stats=(5744, True, 5.744)"
)


class FakeProbe:
    def __init__(self):
        now = time.monotonic()

        self.stats = (5744, True, 5.744)
        self.stats_stamp = now
        self.pose = (0.2089, 0.0)
        self.pos_stamp = now

        self.cmd = (0.0, 0.0)
        self.cmd_stamp = now
        self.limits_breached = False

        self.guard_authorized = True
        self.watchdog_monitor = True
        self.positive_cmd = True

        self.msg = {
            "trial": {
                "trial_phase": "DONE",
                "trial_reason": "TARGET_REACHED",
                "request_state": "HOLD",
            },
            "geofence": {
                "action": "COMPLETE",
                "reason": "TARGET_REACHED",
                "trial_active": False,
            },
            "guard": {
                "guard_state": "BLOCKED",
                "permitted_linear_m_s": 0.0,
                "permitted_angular_rad_s": 0.0,
            },
        }

        self.ages = {}

    def age(self, key):
        return self.ages.get(key, 0.1)

    def require_paused(self):
        if self.stats[1] is not True:
            raise RuntimeError("Gazebo non PAUSED")


def accepted(probe, error=ERROR, n=50):
    before = (
        probe.pos_stamp - 1.0
        if probe.pos_stamp is not None
        else None
    )
    return terminal_step_stopped(
        probe, RuntimeError(error), before, n
    )


def test_stop_nominale_interrompe_sei_step():
    assert accepted(FakeProbe()) is True


@pytest.mark.parametrize("error", [
    "Errore generico ControlWorld",
    ERROR.replace("ACK=True", "ACK=False"),
    ERROR.replace("PAUSED_TARGET=False", "PAUSED_TARGET=True"),
    ERROR.replace("5744, True", "5700, True"),
    ERROR.replace("5744, True", "5751, True"),
])
def test_timeout_non_valido(error):
    assert accepted(FakeProbe(), error) is False


@pytest.mark.parametrize("case", [
    "x_over",
    "x_under",
    "trial",
    "geofence",
    "guard",
    "cmd",
    "pause",
    "odom",
    "authorization",
    "stale_guard",
    "limits",
])
def test_evidenza_stop_incompleta(case):
    p = FakeProbe()

    if case == "x_over":
        p.pose = (0.216, 0.0)
    elif case == "x_under":
        p.pose = (0.195, 0.0)
    elif case == "trial":
        p.msg["trial"]["trial_reason"] = "TIME_LIMIT"
    elif case == "geofence":
        p.msg["geofence"]["action"] = "PAUSE"
    elif case == "guard":
        p.msg["guard"]["permitted_linear_m_s"] = 0.1
    elif case == "cmd":
        p.cmd = (0.1, 0.0)
    elif case == "pause":
        p.stats = (5744, False, 5.744)
    elif case == "odom":
        p.pos_stamp = None
    elif case == "authorization":
        p.guard_authorized = False
    elif case == "stale_guard":
        p.ages["guard"] = 2.0
    elif case == "limits":
        p.limits_breached = True

    assert accepted(p) is False
