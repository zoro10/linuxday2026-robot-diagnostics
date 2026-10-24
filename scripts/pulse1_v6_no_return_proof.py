"""S1-specific positive evidence for valid full-LiDAR scans with no returns.

This helper is independent of ROS; never creates a motion authorization.
A no-return beam is only free *within the calibrated sensor range*.
The V6 contract still checks recent observer data, swept-envelope geometry,
and every sector. Unknown / NaN / missing scans remain denied.
"""
import math
import hashlib
from functools import lru_cache
from pathlib import Path

# Calibrated from simulation/models/pulse1/model_demo2.sdf.
# The front and rear GPU lidars each cover 180 degrees and point oppositely;
# full scans together cover the world azimuth, including lateral directions
# omitted from the 3-sector aggregation.
_SPECS = {'front': (180, 12.0), 'rear': (90, 6.0)}
_MIN_RANGE = 0.12
_HALF_FOV = math.pi / 2
# Only the exact fixed Gazebo S1 scene and calibrated model are approved for
# the special all-no-return case. A changed world/model fails closed until
# re-calibrated and reviewed; these hashes are from the user's supplied SDFs.
_S1_SDF_SHA256 = {
    'simulation/models/pulse1/model_demo2.sdf':
        '614f689a4ac92c693361151938e2d2aaf99305ae8a4ade4776733bb52951da41',
    'simulation/worlds/pulse1_s1.sdf':
        '19c7d60d01395fcc45a6027f25f25ec95710cf195b9dd4d78416764cd8f4ec93',
}


@lru_cache(maxsize=1)
def calibrated_s1_files_present():
    root = Path(__file__).resolve().parent.parent
    try:
        return all(hashlib.sha256((root / rel).read_bytes()).hexdigest() == digest
                   for rel, digest in _S1_SDF_SHA256.items())
    except (OSError, ValueError):
        return False


def _metadata_ok(name, count, angle_min, angle_inc, range_min, range_max):
    if name not in _SPECS or type(count) is not int:
        return False
    expected_count, expected_max = _SPECS[name]
    if count != expected_count or not all(
        type(x) in (int, float) and math.isfinite(x)
        for x in (angle_min, angle_inc, range_min, range_max)
    ):
        return False
    return (
        abs(angle_min + _HALF_FOV) < 0.002
        and abs(angle_min + (count - 1)*angle_inc - _HALF_FOV) < 0.002
        and angle_inc > 0
        and abs(range_min - _MIN_RANGE) < 0.002
        and abs(range_max - expected_max) < 0.002
    )


def make_no_return_certificate(name, ranges, angle_min, angle_inc,
                               range_min, range_max):
    """Attest *all* raw beams are +inf and scan calibration is exact."""
    if ranges is None:
        return None
    count = len(ranges)
    if not _metadata_ok(name, count, angle_min, angle_inc,
                        range_min, range_max):
        return None
    if not all(type(r) in (float, int) and math.isinf(r) and r > 0
               for r in ranges):
        return None
    return {'kind': 'S1_FULL_SCAN_POSITIVE_INFINITY', 'sensor': name,
            'count': count, 'angle_min': float(angle_min),
            'angle_inc': float(angle_inc), 'range_min': float(range_min),
            'range_max': float(range_max)}


def verified_no_return_certificate(name, c):
    """Fail closed unless full-scan metadata matches the known S1 hardware."""
    return (calibrated_s1_files_present()
            and isinstance(c, dict)
            and c.get('kind') == 'S1_FULL_SCAN_POSITIVE_INFINITY'
            and c.get('sensor') == name
            and _metadata_ok(name, c.get('count'), c.get('angle_min'),
                             c.get('angle_inc'), c.get('range_min'),
                             c.get('range_max')))


# Positive evidence carrying the *entire* calibrated scan, not just the
# 3-sector minima. JSON null represents a validated +infinity reading.
# Unlike the all-no-return certificate, finite returns are permitted but must
# be examined individually by the independent turning contract.
def make_full_scan_rays(name, ranges, angle_min, angle_inc,
                        range_min, range_max):
    if ranges is None or not _metadata_ok(name, len(ranges), angle_min,
                                           angle_inc, range_min, range_max):
        return None
    vals = []
    for value in ranges:
        if type(value) not in (int, float):
            return None
        if value == math.inf:
            vals.append(None)
        elif (math.isfinite(value) and range_min <= value <= range_max):
            vals.append(float(value))
        else:
            # NaN, -inf, below minimum and beyond maximum are not free space.
            return None
    return {'kind': 'S1_FULL_SCAN_RANGES_V1', 'sensor': name,
            'count': len(vals), 'angle_min': float(angle_min),
            'angle_inc': float(angle_inc), 'range_min': float(range_min),
            'range_max': float(range_max), 'ranges': vals}


def verified_full_scan_rays(name, proof):
    """Verify immutable S1 SDFs, scan metadata and every raw beam field."""
    if not calibrated_s1_files_present() or not isinstance(proof, dict):
        return False
    if (proof.get('kind') != 'S1_FULL_SCAN_RANGES_V1'
            or proof.get('sensor') != name
            or not _metadata_ok(name, proof.get('count'),
                                proof.get('angle_min'), proof.get('angle_inc'),
                                proof.get('range_min'), proof.get('range_max'))):
        return False
    rays = proof.get('ranges')
    if not isinstance(rays, list) or len(rays) != proof['count']:
        return False
    low, high = proof['range_min'], proof['range_max']
    return all(value is None or (type(value) in (int, float)
                   and math.isfinite(value) and low <= value <= high)
               for value in rays)
