#!/usr/bin/env python3
"""Navigatore AVOID per prova controllata di rotazione."""

import argparse
import json
import os
from pathlib import Path
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from pulse1_turn_logic import turn_step
from pulse1_turn_request import make_turn_request


def observer_ready(data, age):
    return (
        isinstance(data, dict)
        and isinstance(age, (int, float))
        and math.isfinite(age)
        and 0 <= age <= 1.5
        and data.get("source") == "avoid_observer"
        and data.get("mode") == "AVOID"
        and data.get("state") == "SENSOR_READY"
        and data.get("motor_commands") is False
        and all(
            data.get(name) is True
            for name in (
                "odom_ok", "lidar_ok",
                "rear_lidar_ok", "imu_ok",
            )
        )
    )


def clearance_from_observer(data):
    """Stima prudenziale, NON certificazione anticollisione.

    Distanza minima da entrambe le scansioni, ridotta
    di 1.10 m per tenere conto dell'offset dei LiDAR.
    """

    if not isinstance(data, dict):
        return None

    readings = data.get("readings")
    if not isinstance(readings, dict):
        return None

    distances = []

    for sensor in ("front", "rear"):
        reading = readings.get(sensor)

        if not isinstance(reading, dict):
            return None

        sectors = reading.get("sectors")
        if not isinstance(sectors, dict):
            return None

        for name in ("front", "left", "right"):
            sector = sectors.get(name)

            if not isinstance(sector, dict):
                return None

            if sector.get("clear") is True:
                distances.append(math.inf)
                continue

            value = sector.get("distance_m")

            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or value <= 0
            ):
                return None

            distances.append(value)

    minimum = min(distances)

    # Senza alcun ritorno finito non deduciamo
    # una distanza numerica garantita.
    if not math.isfinite(minimum):
        return None

    return max(0.0, minimum - 1.10)


class TurnNavigator(Node):

    def __init__(self, armed=False):
        super().__init__("pulse1_turn_navigator")
        self.armed = armed

        self.observer = None
        self.observer_at = None
        self.pose = None
        self.pose_at = None
        self.imu_at = None
        self.imu_stamp_s = None
        self.gyro_z = None

        self.start_pose = None
        self.started_at = None
        self.gyro_integral = 0.0
        self.finished = None

        self.sub_observer = self.create_subscription(
            String, "/pulse1/avoid_observer",
            self.on_observer, 10,
        )

        self.sub_odom = self.create_subscription(
            Odometry, "/model/pulse1_proto/odometry",
            self.on_odom, qos_profile_sensor_data,
        )

        self.sub_imu = self.create_subscription(
            Imu, "/pulse1/imu",
            self.on_imu, qos_profile_sensor_data,
        )

        self.output = self.create_publisher(
            String, "/pulse1/avoid_request", 10,
        )

        self.create_timer(0.1, self.control)

    def on_observer(self, msg):
        try:
            value = json.loads(msg.data)
            self.observer = value if isinstance(value, dict) else None
        except (ValueError, TypeError):
            self.observer = None

        self.observer_at = time.monotonic()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        numbers = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm = sum(v*v for v in (q.x, q.y, q.z, q.w))

        if (
            not all(math.isfinite(v) for v in numbers)
            or not 0.5 <= norm <= 1.5
        ):
            self.pose_at = None
            return

        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z),
        )

        self.pose = (p.x, p.y, yaw)
        self.pose_at = time.monotonic()

    def on_imu(self, msg):
        now = time.monotonic()
        g = msg.angular_velocity
        a = msg.linear_acceleration

        values = (g.x, g.y, g.z, a.x, a.y, a.z)

        if not all(math.isfinite(v) for v in values):
            self.imu_at = None
            self.imu_stamp_s = None
            self.gyro_z = None
            if self.started_at is not None:
                self.finished = ("ABORT", "INVALID_IMU")
            return

        stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) / 1e9
        )

        if not math.isfinite(stamp) or stamp < 0:
            self.imu_at = None
            self.imu_stamp_s = None
            self.gyro_z = None
            if self.started_at is not None:
                self.finished = ("ABORT", "INVALID_IMU_STAMP")
            return

        if self.imu_stamp_s is not None:
            dt = stamp - self.imu_stamp_s

            if dt <= 0 or dt > 0.35:
                self.imu_at = None
                self.imu_stamp_s = None
                self.gyro_z = None
                if self.started_at is not None:
                    self.finished = ("ABORT", "IMU_STAMP_GAP")
                return

            if self.started_at is not None:
                self.gyro_integral += (
                    0.5 * (self.gyro_z + g.z) * dt
                )

        self.gyro_z = g.z
        self.imu_stamp_s = stamp
        self.imu_at = now

    def control(self):
        now = time.monotonic()

        # La prova armata può iniziare soltanto
        # dalla posizione iniziale dello scenario S1.
        if (
            self.armed
            and self.started_at is None
            and self.pose is not None
            and (
                math.hypot(self.pose[0], self.pose[1]) > 0.30
                or abs(self.pose[2]) > 0.20
            )
        ):
            self.finished = ("ABORT", "UNSAFE_START_POSE")

        if self.finished is not None:
            state, reason = self.finished
            angular = 0.0
            progress = 0.0
        else:
            observer_age = (
                now - self.observer_at
                if self.observer_at is not None else None
            )

            ready = all((
                observer_ready(self.observer, observer_age),
                self.pose_at is not None,
                self.imu_at is not None,
                self.gyro_z is not None,
                self.pose_at is not None
                and now - self.pose_at <= 0.6,
                self.imu_at is not None
                and now - self.imu_at <= 0.5,
            ))

            clearance = (
                clearance_from_observer(self.observer)
                if ready else None
            )

            if not ready or clearance is None:
                if self.started_at is not None:
                    self.finished = ("ABORT", "DATA_LOST")
                    state, reason = self.finished
                else:
                    state, reason = "HOLD", "WAIT_DATA"

                angular = 0.0
                progress = 0.0

            else:
                if self.started_at is None:
                    self.started_at = now
                    self.start_pose = self.pose
                    self.gyro_integral = 0.0

                decision = turn_step(
                    start_pose=self.start_pose,
                    pose=self.pose,
                    integrated_gyro_rad=self.gyro_integral,
                    gyro_z=self.gyro_z,
                    elapsed_s=now-self.started_at,
                    sensors_ok=True,
                    clearance_m=clearance,
                    direction="LEFT",
                )

                state = decision["state"]
                reason = decision["reason"]
                angular = decision["angular_rad_s"]
                progress = decision["progress_rad"]

                if state in ("DONE", "ABORT"):
                    self.finished = (state, reason)
                    angular = 0.0

        message = {
            "source": "turn_shadow",
            "mode": "AVOID",
            "state": state,
            "reason": reason,
            "shadow_angular_rad_s": angular,
            "progress_rad": progress,
            "proposed_linear_m_s": 0.0,
            "proposed_angular_rad_s": 0.0,
            "motor_commands": False,
        }

        # La decisione interna non è un comando motore.
        # Serve anche un'autorizzazione esplicita.
        observer_age = (
            now - self.observer_at
            if self.observer_at is not None else None
        )

        sensors_ready = all((
            observer_ready(self.observer, observer_age),
            self.pose_at is not None,
            self.imu_at is not None,
            self.pose_at is not None
            and now - self.pose_at <= 0.6,
            self.imu_at is not None
            and now - self.imu_at <= 0.5,
        ))

        flags = {
            name: sensors_ready
            for name in (
                "odom_ok", "lidar_ok",
                "rear_lidar_ok", "imu_ok",
            )
        }

        request = make_turn_request(
            {
                "state": state,
                "linear_m_s": 0.0,
                "angular_rad_s": angular,
            },
            flags,
            armed=self.armed,
        )

        request["turn_state"] = state
        request["turn_reason"] = reason
        request["turn_progress_rad"] = progress

        output = String()
        output.data = json.dumps(request, allow_nan=False)
        self.output.publish(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--armed", action="store_true")
    args = parser.parse_args()

    if args.armed:
        if os.environ.get("PULSE1_AVOID_TURN_ARM") != "YES":
            raise SystemExit("Rotazione AVOID non armata")

        metadata = (
            Path(__file__).resolve().parent.parent
            / "runtime/pulse1_world.json"
        )

        try:
            world = json.loads(metadata.read_text())
        except (OSError, ValueError):
            world = {}

        if (
            world.get("scenario") != "s1"
            or world.get("navigation_mode") != "AVOID"
        ):
            raise SystemExit("Missione S1/AVOID non confermata")

    rclpy.init(args=[])
    node = TurnNavigator(armed=args.armed)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
