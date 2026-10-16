import math
import time
import unittest
from types import SimpleNamespace

from pulse1_turn_shadow import TurnShadow
from pulse1_turn_navigator import TurnNavigator


def imu(sec=10, nanosec=100_000_000, gyro=0.1):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=sec,
                nanosec=nanosec,
            )
        ),
        angular_velocity=SimpleNamespace(
            x=0.0, y=0.0, z=gyro
        ),
        linear_acceleration=SimpleNamespace(
            x=0.0, y=0.0, z=9.81
        ),
    )


def fake_controller():
    now = time.monotonic()
    return SimpleNamespace(
        started_at=now - 1.0,
        imu_at=now - 1.0,
        imu_stamp_s=10.0,
        gyro_z=0.1,
        gyro_integral=0.0,
        finished=None,
    )


class ImuClockTests(unittest.TestCase):

    def test_integrates_simulation_time(self):
        for cls in (TurnShadow, TurnNavigator):
            with self.subTest(controller=cls.__name__):
                node = fake_controller()

                # Simulazione avanzata di 0.1 s;
                # tempo reale trascorso di circa 1 s.
                cls.on_imu(node, imu())

                self.assertIsNone(node.finished)
                self.assertAlmostEqual(
                    node.gyro_integral, 0.01, places=5
                )

    def test_large_simulation_gap_aborts(self):
        for cls in (TurnShadow, TurnNavigator):
            with self.subTest(controller=cls.__name__):
                node = fake_controller()
                cls.on_imu(
                    node, imu(sec=10, nanosec=600_000_000)
                )
                self.assertEqual(
                    node.finished,
                    ("ABORT", "IMU_STAMP_GAP"),
                )

    def test_backward_timestamp_aborts(self):
        for cls in (TurnShadow, TurnNavigator):
            with self.subTest(controller=cls.__name__):
                node = fake_controller()
                cls.on_imu(node, imu(sec=9))
                self.assertEqual(
                    node.finished,
                    ("ABORT", "IMU_STAMP_GAP"),
                )

    def test_invalid_gyro_aborts(self):
        for cls in (TurnShadow, TurnNavigator):
            with self.subTest(controller=cls.__name__):
                node = fake_controller()
                cls.on_imu(node, imu(gyro=math.nan))
                self.assertEqual(
                    node.finished,
                    ("ABORT", "INVALID_IMU"),
                )


if __name__ == "__main__":
    unittest.main()
