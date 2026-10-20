import pytest
from pulse1_nominal_stop_v5 import MAX_X, step_budget


def test_initial_budget_is_bounded():
    assert step_budget(0.0, 0.001) == 50


def test_sensor_sized_steps_near_target():
    for x in (0.190, 0.1932, 0.1972, 0.198):
        assert step_budget(x, 0.001) == 50


def test_near_target_step_keeps_safety_margin():
    x = 0.1972
    n = step_budget(x, 0.001)
    worst_case = x + n * 0.001 * 0.10 * 1.5
    assert worst_case < 0.215


def test_step_budget_stays_under_prudential_limit():
    for x in (0.0, 0.18, 0.195, 0.200, 0.212):
        n = step_budget(x, 0.001)
        assert 1 <= n <= 50
        assert x + n * 0.001 * 0.10 * 1.5 <= MAX_X + 1e-10


@pytest.mark.parametrize('x', [float('nan'), 0.215, 0.22, -float('inf')])
def test_invalid_or_out_of_margin_pose_refused(x):
    with pytest.raises(RuntimeError):
        step_budget(x, 0.001)


@pytest.mark.parametrize('dt', [0.0, -0.001, 0.002, float('nan')])
def test_invalid_sim_timestep_refused(dt):
    with pytest.raises(RuntimeError):
        step_budget(0.0, dt)


def test_new_odometry_required():
    from pulse1_nominal_stop_v5 import require_new_odometry

    require_new_odometry(10.0, 10.1, 0.05)


@pytest.mark.parametrize("before,after,age", [
    (10.0, 10.0, 0.05),
    (10.0, 9.9, 0.05),
    (10.0, None, None),
    (10.0, 10.1, 0.8),
])
def test_stale_odometry_rejected(before, after, age):
    from pulse1_nominal_stop_v5 import require_new_odometry

    with pytest.raises(RuntimeError):
        require_new_odometry(before, after, age)
