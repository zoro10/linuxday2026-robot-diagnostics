import pytest

from pulse1_s1_geofence_logic_v2 import GeofenceV2


def evaluate(
    g, *,
    x=0.20,
    active=False,
    done=True,
    guard_state="BLOCKED",
    speed=0.0,
    elapsed=8.1,
    paused=True,
    now=8.1,
):
    guard = {
        "source": "motor_guard_live",
        "mode": "AVOID",
        "motor_commands": True,
        "guard_state": guard_state,
        "permitted_linear_m_s": speed,
        "permitted_angular_rad_s": 0.0,
    }

    return g.step(
        active=active,
        completion_requested=done,
        pose=(x, 0.0, 0.0),
        pose_age_s=0.0,
        guard=guard,
        guard_age_s=0.0,
        cmd=(speed, 0.0),
        cmd_age_s=0.0,
        elapsed_s=elapsed,
        world_paused=paused,
        world_age_s=0.0,
        now_s=now,
    )


def enter_stopping():
    g = GeofenceV2()

    r = evaluate(
        g, x=0.19,
        active=True,
        done=False,
        guard_state="AUTHORIZED",
        speed=0.10,
        elapsed=7.50,
        paused=False,
        now=7.50,
    )
    assert r["action"] == "MONITOR"

    r = evaluate(
        g,
        elapsed=7.90,
        paused=False,
        now=7.90,
    )
    assert r["action"] == "STOPPING"

    return g


def test_stopping_can_complete_after_drive_timeout():
    g = enter_stopping()

    assert evaluate(g) == {
        "action": "COMPLETE",
        "reason": "TARGET_REACHED",
    }


def test_stopping_still_has_two_second_deadline():
    g = enter_stopping()

    r = evaluate(
        g,
        elapsed=10.1,
        now=10.1,
    )

    assert r == {
        "action": "PAUSE",
        "reason": "STOP_TIMEOUT",
    }


def test_drive_still_has_eight_second_limit():
    g = GeofenceV2()

    r = evaluate(
        g,
        x=0.19,
        active=True,
        done=False,
        guard_state="AUTHORIZED",
        speed=0.10,
        elapsed=8.0,
        paused=False,
        now=8.0,
    )

    assert r == {
        "action": "PAUSE",
        "reason": "TRIAL_TIMEOUT",
    }


def test_emergency_limit_applies_during_stopping():
    g = enter_stopping()

    r = evaluate(g, x=0.22)

    assert r == {
        "action": "PAUSE",
        "reason": "POSITION_LIMIT",
    }


def test_nonzero_command_blocked_during_stopping():
    g = enter_stopping()

    r = evaluate(g, speed=0.01)

    assert r["action"] == "PAUSE"


def test_lost_stop_request_fails_closed():
    g = enter_stopping()

    r = evaluate(g, done=False)

    assert r == {
        "action": "PAUSE",
        "reason": "STOP_REQUEST_LOST",
    }


@pytest.mark.parametrize("elapsed", [
    None,
    float("nan"),
    -1.0,
])
def test_invalid_elapsed_fails_closed(elapsed):
    g = enter_stopping()

    r = evaluate(g, elapsed=elapsed)

    assert r == {
        "action": "PAUSE",
        "reason": "INVALID_STOP_ELAPSED",
    }
