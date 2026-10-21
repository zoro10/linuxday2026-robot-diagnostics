"""Pure V6 heartbeat validator, independent of ROS and old V5 supervisor."""
import math


def health(status, age):
    if (not isinstance(status, dict) or type(age) not in (int, float)
            or not math.isfinite(age) or not 0 <= age <= .8):
        return 'GUARD_LOST'
    if (status.get('source') != 'motor_guard_live'
            or status.get('mode') != 'AVOID'
            or status.get('v6_armed') is not True
            or status.get('motor_commands') is not True
            or status.get('cmd_vel_exclusive') is not True
            or status.get('emergency_latched') is True
            or status.get('guard_state') not in ('BLOCKED', 'AUTHORIZED')):
        return 'GUARD_UNSAFE'
    return 'HEALTHY'
