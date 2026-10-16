import math
import unittest

from pulse1_avoid_sectors import sector_distances


def scan(ranges, angle_min=-math.pi / 2):
    increment = math.pi / (len(ranges) - 1)

    return sector_distances(
        ranges,
        angle_min,
        increment,
        0.12,
        12.0,
    )


class AvoidSectorTests(unittest.TestCase):

    def test_all_clear(self):
        result = scan([math.inf] * 181)

        self.assertTrue(all(
            value == math.inf
            for value in result.values()
        ))

    def test_front_obstacle(self):
        ranges = [math.inf] * 181
        ranges[90] = 1.18

        result = scan(ranges)

        self.assertEqual(result["front"], 1.18)
        self.assertEqual(result["left"], math.inf)
        self.assertEqual(result["right"], math.inf)

    def test_left_obstacle(self):
        ranges = [math.inf] * 181
        ranges[120] = 0.8

        result = scan(ranges)

        self.assertEqual(result["left"], 0.8)
        self.assertEqual(result["right"], math.inf)

    def test_right_obstacle(self):
        ranges = [math.inf] * 181
        ranges[60] = 0.9

        result = scan(ranges)

        self.assertEqual(result["right"], 0.9)
        self.assertEqual(result["left"], math.inf)

    def test_invalid_measurement(self):
        ranges = [math.inf] * 181
        ranges[90] = float("nan")

        result = scan(ranges)

        self.assertIsNone(result["front"])

    def test_partial_field_of_view(self):
        result = sector_distances(
            [math.inf] * 41,
            -0.2,
            0.01,
            0.12,
            12.0,
        )

        self.assertIsNone(result["left"])
        self.assertIsNone(result["right"])

    def test_invalid_metadata(self):
        result = sector_distances(
            [math.inf] * 181,
            float("nan"),
            0.01,
            0.12,
            12.0,
        )

        self.assertTrue(all(
            value is None
            for value in result.values()
        ))

    def test_rear_scan_resolution(self):
        result = scan([math.inf] * 90)

        self.assertEqual(result["front"], math.inf)
        self.assertEqual(result["left"], math.inf)
        self.assertEqual(result["right"], math.inf)


if __name__ == "__main__":
    unittest.main()
