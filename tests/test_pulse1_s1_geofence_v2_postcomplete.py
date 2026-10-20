import pytest
from pulse1_s1_geofence_logic_v2 import GeofenceV2


def guard(state, speed=0.0):
    return {
        "source": "motor_guard_live",
        "mode": "AVOID",
        "motor_commands": True,
        "guard_state": state,
        "permitted_linear_m_s": speed,
        "permitted_angular_rad_s": 0.0,
    }


def step(g, **overrides):
    data = {
        "active": False,
        "completion_requested": False,
        "pose": (0.20, 0.0, 0.0),
        "pose_age_s": 0.0,
        "guard": guard("STALE_REQUEST"),
        "guard_age_s": 0.0,
        "cmd": (0.0, 0.0),
        "cmd_age_s": 0.0,
        "elapsed_s": 2.0,
        "world_paused": True,
        "world_age_s": 0.0,
        "now_s": 2.8,
    }
    data.update(overrides)
    return g.step(**data)


def completed_geofence():
    g = GeofenceV2()

    assert step(
        g,
        active=True,
        pose=(0.10, 0.0, 0.0),
        guard=guard("AUTHORIZED", 0.10),
        cmd=(0.10, 0.0),
        world_paused=False,
        now_s=1.0,
    )["action"] == "MONITOR"

    assert step(
        g,
        completion_requested=True,
        guard=guard("BLOCKED"),
        world_paused=False,
        now_s=2.0,
    )["action"] == "STOPPING"

    assert step(
        g,
        completion_requested=True,
        guard=guard("BLOCKED"),
        now_s=2.1,
    )["action"] == "COMPLETE"

    return g


def test_complete_survives_expired_trial_request():
    g = completed_geofence()
    result = step(g, guard=guard("STALE_REQUEST"))
    assert result == {
        "action": "COMPLETE",
        "reason": "TARGET_REACHED",
    }
    assert g.alarm is None


def test_complete_remains_with_blocked_guard():
    g = completed_geofence()
    assert step(g, guard=guard("BLOCKED"))["action"] == "COMPLETE"


@pytest.mark.parametrize("overrides", [
    {"world_paused": False},
    {"world_age_s": 1.0},
    {"guard_age_s": 1.0},
    {"cmd_age_s": 1.0},
    {"cmd": (0.01, 0.0)},
    {"guard": guard("AUTHORIZED")},
    {"pose": (0.22, 0.0, 0.0)},
    {"pose": (0.19, 0.0, 0.0)},
    {"pose": (0.20, 0.11, 0.0)},
    {"active": True},
])
def test_unsafe_after_complete_fails_closed(overrides):
    g = completed_geofence()
    result = step(g, **overrides)
    assert result["action"] == "PAUSE"
    assert result["reason"] == "UNSAFE_AFTER_COMPLETE"


def test_stale_request_cannot_confirm_initial_stop():
    g = GeofenceV2()

    assert step(
        g,
        active=True,
        pose=(0.10, 0.0, 0.0),
        guard=guard("AUTHORIZED", 0.10),
        cmd=(0.10, 0.0),
        world_paused=False,
        now_s=1.0,
    )["action"] == "MONITOR"

    assert step(
        g,
        completion_requested=True,
        guard=guard("BLOCKED"),
        world_paused=False,
        now_s=2.0,
    )["action"] == "STOPPING"

    assert step(
        g,
        completion_requested=True,
        guard=guard("STALE_REQUEST"),
        now_s=2.1,
    )["action"] == "PAUSE"


def test_complete_with_frozen_odometry():
    g = completed_geofence()

    result = step(
        g,
        pose_age_s=5.0,
        guard=guard("STALE_REQUEST"),
        world_paused=True,
        world_age_s=0.0,
    )

    assert result["action"] == "COMPLETE"
    assert g.alarm is None


@pytest.mark.parametrize("overrides", [
    {"world_paused": False},
    {"world_age_s": 1.0},
    {"cmd_age_s": 1.0},
    {"guard_age_s": 1.0},
    {"cmd": (0.01, 0.0)},
    {"guard": guard("AUTHORIZED")},
    {"pose": (0.22, 0.0, 0.0)},
    {"pose": None},
    {"pose_age_s": None},
])
def test_frozen_odometry_still_requires_safe_state(overrides):
    g = completed_geofence()

    data = {"pose_age_s": 5.0}
    data.update(overrides)

    result = step(g, **data)

    assert result["action"] == "PAUSE"
    assert result["reason"] == "UNSAFE_AFTER_COMPLETE"


def test_stale_odometry_cannot_confirm_initial_stop():
    g = GeofenceV2()

    assert step(
        g,
        active=True,
        pose=(0.10, 0.0, 0.0),
        guard=guard("AUTHORIZED", 0.10),
        cmd=(0.10, 0.0),
        world_paused=False,
        now_s=1.0,
    )["action"] == "MONITOR"

    assert step(
        g,
        completion_requested=True,
        guard=guard("BLOCKED"),
        world_paused=False,
        now_s=2.0,
    )["action"] == "STOPPING"

    result = step(
        g,
        completion_requested=True,
        guard=guard("BLOCKED"),
        pose_age_s=5.0,
        now_s=2.1,
    )

    assert result["action"] == "PAUSE"
