import pytest
import pulse1_s1_trial_gate as gate_module

from pulse1_s1_trial_gate import TrialGate
from pulse1_s1_geofence_logic import Geofence
from pulse1_s1_geofence_logic_v2 import GeofenceV2


@pytest.fixture(autouse=True)
def mock_request(monkeypatch):
    def request(decision, sensors, pose, *, armed=False):
        return {
            "state": "GO_TO_GOAL" if armed else "HOLD"
        }

    monkeypatch.setattr(
        gate_module, "make_approach_request", request
    )


def gate_tick(g, t, x=0.0):
    return g.step(
        {}, {}, (x, 0.0, 0.0), t, armed=True
    )


def gf_tick(g, t, x=0.10, active=True,
            done=False, paused=False):

    guard = {
        "source": "motor_guard_live",
        "mode": "AVOID",
        "motor_commands": True,
        "guard_state": (
            "AUTHORIZED" if active else "BLOCKED"
        ),
        "permitted_linear_m_s": 0.0,
        "permitted_angular_rad_s": 0.0,
    }

    return g.step(
        active=active,
        completion_requested=done,
        pose=(x, 0.0, 0.0),
        pose_age_s=0.0,
        guard=guard,
        guard_age_s=0.0,
        cmd=(0.0, 0.0),
        cmd_age_s=0.0,
        elapsed_s=t,
        world_paused=paused,
        world_age_s=0.0,
        now_s=t,
    )


def test_default_gate_timeout():
    assert gate_tick(TrialGate(), 8.0)[
        "trial_reason"
    ] == "TIME_LIMIT"


def test_v5_gate_drive_and_deadline():
    g = TrialGate(max_duration_s=16.0)

    assert gate_tick(g, 12.0)["trial_phase"] == "DRIVE"
    assert gate_tick(g, 16.0)["trial_reason"] == "TIME_LIMIT"
    assert gate_tick(g, 1.0)["trial_reason"] == "TIME_LIMIT"


def test_v5_gate_target():
    g = TrialGate(max_duration_s=16.0)

    assert gate_tick(g, 12.0)["trial_phase"] == "DRIVE"

    r = gate_tick(g, 15.9, x=0.20)
    assert r["trial_phase"] == "DONE"
    assert r["trial_reason"] == "TARGET_REACHED"


def test_v1_timeout_unchanged():
    g = Geofence()
    assert g.step(
        active=True, elapsed_s=8.0
    )["reason"] == "TRIAL_TIMEOUT"


def test_v2_default_timeout_unchanged():
    g = GeofenceV2()
    assert gf_tick(g, 8.0)["reason"] == "TRIAL_TIMEOUT"


def test_v5_geofence_drive_and_deadline():
    g = GeofenceV2(max_drive_s=16.0)

    assert gf_tick(g, 12.0)["action"] == "MONITOR"

    r = gf_tick(g, 16.0)
    assert r["action"] == "PAUSE"
    assert r["reason"] == "TRIAL_TIMEOUT"


def test_v5_stop_after_drive_deadline():
    g = GeofenceV2(max_drive_s=16.0)

    assert gf_tick(
        g, 15.5, x=0.19
    )["action"] == "MONITOR"

    assert gf_tick(
        g, 15.9, x=0.20,
        active=False, done=True
    )["action"] == "STOPPING"

    assert gf_tick(
        g, 16.1, x=0.20,
        active=False, done=True, paused=True
    )["action"] == "COMPLETE"


def test_v5_emergency_limit():
    g = GeofenceV2(max_drive_s=16.0)

    r = gf_tick(g, 12.0, x=0.22)

    assert r["action"] == "PAUSE"
    assert r["reason"] == "POSITION_LIMIT"


@pytest.mark.parametrize("value", [
    17.0, 100.0, float("nan"), True
])
def test_unsupported_duration_rejected(value):
    with pytest.raises(ValueError):
        TrialGate(max_duration_s=value)

    with pytest.raises(ValueError):
        GeofenceV2(max_drive_s=value)
