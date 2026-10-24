"""Five-attempt sensor budget for a single V6 transition; no ROS or rearm."""
import math

MAX_REFRESH_ATTEMPTS = 5
# Already established in the V6 stationary-odometry recovery contract.
STATIONARY_DISTANCE_M = 0.005
STATIONARY_YAW_RAD = 0.010


def odom_snapshot(node):
    observer = node.value("observer")
    readings = observer.get("readings") if isinstance(observer, dict) else None
    odom = readings.get("odom") if isinstance(readings, dict) else None
    if not isinstance(odom, dict):
        raise RuntimeError("SENSOR_ODOM_MISSING")
    values = tuple(odom.get(k) for k in ("x", "y", "yaw_rad"))
    if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
        raise RuntimeError("SENSOR_ODOM_INVALID")
    return values


def ensure_stationary(reference, current):
    distance = math.hypot(current[0] - reference[0],
                          current[1] - reference[1])
    yaw = math.atan2(math.sin(current[2] - reference[2]),
                     math.cos(current[2] - reference[2]))
    if distance > STATIONARY_DISTANCE_M or abs(yaw) > STATIONARY_YAW_RAD:
        raise RuntimeError("SENSOR_REFRESH_ROVER_MOVED")


def refresh_with_budget(node, kind, budget, *, phase, reference_odom=None):
    """Retry ONLY known recoverable sensor freshness issues.

    The underlying refresh is fail-closed: it only performs acknowledged
    ControlWorld steps while the previous nominal STOP is independently
    certified. Never retry an unacknowledged step or a safety denial.
    Returns (fresh_odometry, number_of_refresh_calls).
    """
    if type(budget) is not int or not 1 <= budget <= MAX_REFRESH_ATTEMPTS:
        raise RuntimeError("SENSOR_REFRESH_BUDGET_EXHAUSTED")
    for attempt in range(1, budget + 1):
        node.wait(node.stopped, "SENSOR_STOP_BEFORE_REFRESH")
        print(f"SENSOR RETRY {phase}: {attempt}/{budget}", flush=True)
        try:
            node.refresh_sensors_before_rearm(kind)
        except RuntimeError as exc:
            if str(exc) != "SENSOR_PREFLIGHT_NO_FRESH_DATA":
                raise
            node.wait(node.stopped, "SENSOR_STOP_AFTER_NO_FRESH_DATA")
            if attempt == budget:
                raise RuntimeError(
                    f"SENSOR_REFRESH_EXHAUSTED:{phase}:NO_FRESH_DATA") from exc
            continue
        node.wait(node.stopped, "SENSOR_STOP_AFTER_REFRESH")
        ready, reason = node.sensor_gate(kind)
        if ready:
            current = odom_snapshot(node)
            if reference_odom is not None:
                ensure_stationary(reference_odom, current)
            print(f"SENSOR RETRY OK {phase}: {attempt}/{budget}", flush=True)
            return current, attempt
        if reason != "OBSERVER_NOT_HEALTHY":
            raise RuntimeError("SENSOR_PREFLIGHT_DENIED:" + str(reason))
        if attempt == budget:
            raise RuntimeError(
                f"SENSOR_REFRESH_EXHAUSTED:{phase}:OBSERVER_NOT_HEALTHY")
    raise RuntimeError("SENSOR_REFRESH_EXHAUSTED")


def check_replan_or_refresh(node, kind, reference_odom, used):
    """Recheck after a potentially slow Gazebo pose read; budget total is 5."""
    node.wait(node.stopped, "STOP_BEFORE_REPLAN_SENSOR_CHECK")
    ready, reason = node.sensor_gate(kind)
    if ready:
        ensure_stationary(reference_odom, odom_snapshot(node))
        print("SENSOR PREFLIGHT REPLAN: STILL FRESH", flush=True)
        return used
    if reason != "OBSERVER_NOT_HEALTHY":
        raise RuntimeError("SENSOR_PREFLIGHT_REPLAN_DENIED:" + str(reason))
    remaining = MAX_REFRESH_ATTEMPTS - used
    if remaining <= 0:
        raise RuntimeError("SENSOR_REFRESH_EXHAUSTED:REPLAN:BUDGET_5")
    _, retried = refresh_with_budget(
        node, kind, remaining, phase="REPLAN", reference_odom=reference_odom)
    return used + retried
