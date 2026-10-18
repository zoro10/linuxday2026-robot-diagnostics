#!/usr/bin/env python3
"""Protezione del primo APPROACH. Nessuna dipendenza ROS."""

import math

from pulse1_s1_approach_request import make_approach_request

STOP_X_M = 0.20
MAX_DURATION_S = 8.0
MAX_START_OFFSET_M = 0.03


def finite(value):
    return (
        type(value) in (int, float)
        and math.isfinite(value)
    )


class TrialGate:
    """Una sola prova: dopo STOP/ABORT non si riparte."""

    def __init__(self):
        self.started = False
        self.terminal_reason = None

    def step(
        self, decision, sensors, pose, elapsed_s,
        *, armed=False
    ):
        def blocked(reason):
            result = make_approach_request(
                decision, sensors, pose, armed=False
            )
            result["trial_phase"] = (
                "DONE"
                if self.terminal_reason == "TARGET_REACHED"
                else "HOLD"
            )
            result["trial_reason"] = reason
            return result

        def abort(reason):
            self.terminal_reason = reason
            return blocked(reason)

        if self.terminal_reason is not None:
            return blocked(self.terminal_reason)

        if armed is not True:
            if self.started:
                return abort("ARM_LOST")
            return blocked("NOT_ARMED")

        if (
            not finite(elapsed_s)
            or elapsed_s < 0
            or elapsed_s >= MAX_DURATION_S
        ):
            return abort("TIME_LIMIT")

        if (
            not isinstance(pose, (tuple, list))
            or len(pose) != 3
            or not all(finite(v) for v in pose)
        ):
            return abort("INVALID_POSE")

        x, y, yaw = pose

        if not self.started and (
            abs(x) > MAX_START_OFFSET_M
            or abs(y) > MAX_START_OFFSET_M
            or abs(yaw) > MAX_START_OFFSET_M
        ):
            return abort("UNSAFE_START_POSE")

        if x >= STOP_X_M:
            self.terminal_reason = "TARGET_REACHED"
            return blocked("TARGET_REACHED")

        request = make_approach_request(
            decision, sensors, pose, armed=True
        )

        if request["state"] != "GO_TO_GOAL":
            return abort("REQUEST_DENIED")

        self.started = True
        request["trial_phase"] = "DRIVE"
        request["trial_reason"] = "LIMITED_FORWARD_TRIAL"

        return request
