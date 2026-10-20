"""Geofence S1 V2: STOP nominale verificato, senza alterare la V1."""

from pulse1_s1_geofence_logic import (
    Geofence, EMERGENCY_X_M, MAX_Y_M, MAX_YAW_RAD, MAX_TRIAL_S, finite,
)

STOP_X_M = 0.20
MAX_STOPPING_S = 2.0
MAX_REPORT_AGE_S = 0.8


class GeofenceV2(Geofence):
    def __init__(self):
        super().__init__()
        self.stopping_since = None
        self.completed = False

    def _alarm(self, reason):
        self.alarm = reason
        return {"action": "PAUSE", "reason": reason}

    @staticmethod
    def _stopped(data, world_paused, world_age_s, *, allow_stale_request=False):
        guard = data.get("guard")
        cmd = data.get("cmd")
        if world_paused is not True or not finite(world_age_s):
            return False
        if not 0 <= world_age_s <= MAX_REPORT_AGE_S:
            return False
        if not isinstance(guard, dict):
            return False
        if not (
            guard.get("source") == "motor_guard_live"
            and guard.get("mode") == "AVOID"
            and guard.get("motor_commands") is True
            and guard.get("guard_state") in (
                ("BLOCKED", "STALE_REQUEST")
                if allow_stale_request else ("BLOCKED",)
            )
        ):
            return False
        if not isinstance(cmd, (tuple, list)) or len(cmd) != 2:
            return False
        speeds = (
            *cmd,
            guard.get("permitted_linear_m_s"),
            guard.get("permitted_angular_rad_s"),
        )
        if not all(finite(v) and abs(v) < 1e-8 for v in speeds):
            return False
        for key in ("guard_age_s", "cmd_age_s"):
            age = data.get(key)
            if not finite(age) or not 0 <= age <= MAX_REPORT_AGE_S:
                return False
        return True

    def step(self, *, completion_requested=False, world_paused=False,
             world_age_s=None, now_s=None, **data):
        if self.alarm is not None:
            return {"action": "PAUSE", "reason": self.alarm}

        if self.completed:
            pose = data.get("pose")
            pose_age_s = data.get("pose_age_s")

            # COMPLETE was entered only after fresh odometry.
            # Gazebo stops publishing odometry while PAUSED.
            # _stopped() must still confirm fresh PAUSED world
            # statistics, fresh guard/cmd and zero velocity.
            safe_pose = (
                isinstance(pose, (tuple, list))
                and len(pose) == 3
                and all(finite(v) for v in pose)
                and finite(pose_age_s)
                and 0 <= pose_age_s
                and STOP_X_M <= pose[0] < EMERGENCY_X_M
                and abs(pose[1]) <= MAX_Y_M
                and abs(pose[2]) <= MAX_YAW_RAD
            )

            if (
                data.get("active") is True
                or not safe_pose
                or not self._stopped(
                    data, world_paused, world_age_s,
                    allow_stale_request=True
                )
            ):
                return self._alarm("UNSAFE_AFTER_COMPLETE")

            return {"action": "COMPLETE", "reason": "TARGET_REACHED"}

        if self.stopping_since is not None:
            if not finite(now_s) or not 0 <= now_s - self.stopping_since <= MAX_STOPPING_S:
                return self._alarm("STOP_TIMEOUT")
            if completion_requested is not True or data.get("active") is True:
                return self._alarm("STOP_REQUEST_LOST")
            # DRIVE has already ended with TARGET_REACHED.
            # STOPPING has its own monotonic 2-second deadline.
            # Preserve every V1 safety check except its DRIVE clock.
            elapsed_s = data.get("elapsed_s")
            if not finite(elapsed_s) or elapsed_s < 0:
                return self._alarm("INVALID_STOP_ELAPSED")

            stop_data = {
                **data,
                "active": True,
                "elapsed_s": min(
                    elapsed_s, MAX_TRIAL_S - 0.001
                ),
            }
            status = super().step(**stop_data)
            if status["action"] != "MONITOR":
                return status
            if data["pose"][0] < STOP_X_M:
                return self._alarm("EARLY_COMPLETION")
            if self._stopped(data, world_paused, world_age_s):
                self.completed = True
                return {"action": "COMPLETE", "reason": "TARGET_REACHED"}
            return {"action": "STOPPING", "reason": "TARGET_REACHED"}

        if completion_requested is not True:
            return super().step(**data)

        if data.get("active") is True or self.engaged is not True:
            return self._alarm("UNEXPECTED_COMPLETION")
        if not finite(now_s):
            return self._alarm("INVALID_STOP_CLOCK")
        status = super().step(**{**data, "active": True})
        if status["action"] != "MONITOR":
            return status
        if not STOP_X_M <= data["pose"][0] < EMERGENCY_X_M:
            return self._alarm("EARLY_COMPLETION")
        self.stopping_since = now_s
        return {"action": "STOPPING", "reason": "TARGET_REACHED"}
