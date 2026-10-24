#!/usr/bin/env python3
"""S1 waypoint planner / shadow-only state machine. No ROS, no motor output.

Geometry uses S1 obstacle metadata and the existing conservative turning
radius. Routes are candidates, NOT a collision-safe motion authorization.
"""

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

from pulse1_navigation_math import turning_envelope, wrap_angle

GOAL = (10.0, 0.0)
START = (0.0, 0.0)
ROUTE_PADDING_M = 0.45  # Added to full turning envelope, not just half-width.
WAYPOINT_TOLERANCE_M = 0.20
OBSERVER_MAX_AGE_S = 0.60
HEADING_TOLERANCE_RAD = 0.12
FLAGS = ('odom_ok', 'lidar_ok', 'rear_lidar_ok', 'imu_ok')


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class Rectangle:
    x: float
    y: float
    size_x: float
    size_y: float

    def __post_init__(self):
        if not all(finite(v) for v in (self.x, self.y, self.size_x, self.size_y)):
            raise ValueError('Geometria non finita')
        if self.size_x <= 0 or self.size_y <= 0:
            raise ValueError('Dimensioni ostacolo non valide')

    def bounds(self, inflation=0.0):
        if not finite(inflation) or inflation < 0:
            raise ValueError('Inflazione non valida')
        return (
            self.x - self.size_x / 2 - inflation,
            self.x + self.size_x / 2 + inflation,
            self.y - self.size_y / 2 - inflation,
            self.y + self.size_y / 2 + inflation,
        )


def s1_obstacle(world):
    if not isinstance(world, dict) or world.get('scenario') != 's1' or world.get('navigation_mode') != 'AVOID':
        raise ValueError('Scenario S1/AVOID richiesto')
    items = world.get('obstacles')
    if not isinstance(items, list) or len(items) != 1:
        raise ValueError('Occorre un solo ostacolo S1 noto')
    item = items[0]
    if not isinstance(item, dict) or item.get('name') != 'pulse1_s1_obstacle':
        raise ValueError('Ostacolo S1 non riconosciuto')
    obstacle = Rectangle(*(item.get(k) for k in ('x', 'y', 'size_x', 'size_y')))
    # This planner is intentionally NOT reusable with arbitrary world geometry.
    expected = (5.0, 0.0, 1.0, 1.4)
    actual = (obstacle.x, obstacle.y, obstacle.size_x, obstacle.size_y)
    if any(abs(v - x) > 1e-6 for v, x in zip(actual, expected)):
        raise ValueError('Geometria S1 diversa dal modello verificato')
    return obstacle


def segment_hits_aabb(start, end, bounds):
    """Segment vs closed, axis-aligned box: touching also counts as collision."""
    x0, y0 = start
    x1, y1 = end
    xmin, xmax, ymin, ymax = bounds
    tmin, tmax = 0.0, 1.0
    for origin, delta, low, high in ((x0, x1-x0, xmin, xmax), (y0, y1-y0, ymin, ymax)):
        if abs(delta) <= 1e-12:
            if low <= origin <= high:
                continue
            return False
        t1, t2 = (low-origin)/delta, (high-origin)/delta
        tmin = max(tmin, min(t1, t2))
        tmax = min(tmax, max(t1, t2))
        if tmin > tmax:
            return False
    return True


# Additional stand-off for the S1 first turning corner.
# Keeps LiDAR TURN clearance unchanged.
S1_WP0_EXTRA_SETBACK_M = 0.27


def build_route(world, side):
    """Return a rectilinear *candidate* outside a conservative S1 keepout."""
    if side not in ('LEFT', 'RIGHT'):
        raise ValueError('Lato consentito: LEFT o RIGHT')
    obs = s1_obstacle(world)
    envelope = turning_envelope()
    xmin, xmax, ymin, ymax = obs.bounds(envelope)
    before_x = xmin - ROUTE_PADDING_M - S1_WP0_EXTRA_SETBACK_M
    after_x = xmax + ROUTE_PADDING_M
    corridor_y = (ymax if side == 'LEFT' else ymin)
    corridor_y += ROUTE_PADDING_M if side == 'LEFT' else -ROUTE_PADDING_M

    if before_x <= 0 or after_x >= GOAL[0]:
        raise ValueError('Spazio insufficiente per ingresso/uscita S1')

    waypoints = (
        (before_x, 0.0),
        (before_x, corridor_y),
        (after_x, corridor_y),
        (after_x, 0.0),
        GOAL,
    )
    points = (START,) + waypoints
    keepout = obs.bounds(envelope)
    if any(segment_hits_aabb(a, b, keepout) for a, b in zip(points, points[1:])):
        raise ValueError('Traiettoria interseca area di rispetto')
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return {
        'scenario': 's1',
        'mode': 'AVOID',
        'side': side,
        'start': list(START),
        'goal': list(GOAL),
        'waypoints': [list(p) for p in waypoints],
        'turning_envelope_m': round(envelope, 4),
        'extra_padding_m': ROUTE_PADDING_M,
        'inflated_obstacle_bounds': [round(v, 4) for v in keepout],
        'path_length_m': round(length, 3),
        'motion_authorized': False,
    }


def valid_observer(data, age_s):
    return (
        finite(age_s) and 0 <= age_s <= OBSERVER_MAX_AGE_S
        and isinstance(data, dict)
        and data.get('source') == 'avoid_observer'
        and data.get('mode') == 'AVOID'
        and data.get('state') == 'SENSOR_READY'
        and data.get('motor_commands') is False
        and all(data.get(k) is True for k in FLAGS)
    )


def valid_pose(pose):
    return isinstance(pose, (list, tuple)) and len(pose) == 3 and all(finite(v) for v in pose)


def observer_pose(data):
    if not isinstance(data, dict):
        return None
    readings = data.get('readings')
    odom = readings.get('odom') if isinstance(readings, dict) else None
    if not isinstance(odom, dict):
        return None
    result = (odom.get('x'), odom.get('y'), odom.get('yaw_rad'))
    return result if valid_pose(result) else None


def front_verified(data):
    """Front sector known: inf/clear is supported; unknown is fail-closed."""
    try:
        sensor = data['readings']['front']['sectors']['front']
    except (KeyError, TypeError):
        return (False, None)
    if not isinstance(sensor, dict):
        return (False, None)
    if sensor.get('clear') is True and sensor.get('distance_m') is None:
        return (True, math.inf)
    distance = sensor.get('distance_m')
    if sensor.get('clear') is False and finite(distance) and distance > 0:
        return (True, float(distance))
    return (False, None)


class WaypointShadow:
    """Progress checker only; never authorizes, publishes or executes motion."""

    def __init__(self, route):
        self.route = route
        self.index = 0
        self.arrived = False

    def evaluate(self, data, age_s):
        result = {
            'source': 'waypoint_shadow',
            'mode': 'AVOID',
            'state': 'HOLD',
            'reason': 'WAIT_SENSORS',
            'side': self.route['side'],
            'waypoint_index': self.index,
            'waypoint_count': len(self.route['waypoints']),
            'target': None,
            'pose': None,
            'distance_m': None,
            'heading_error_rad': None,
            'suggested_action': 'HOLD',
            'proposed_linear_m_s': 0.0,
            'proposed_angular_rad_s': 0.0,
            'motor_commands': False,
            'motion_authorized': False,
        }
        if self.arrived:
            result.update(state='ARRIVED', reason='WAYPOINTS_COMPLETE',
                          waypoint_index=len(self.route['waypoints']))
            return result
        if not valid_observer(data, age_s):
            return result
        pose = observer_pose(data)
        if pose is None:
            result['reason'] = 'INVALID_POSE'
            return result
        known_front, front_m = front_verified(data)
        if not known_front:
            result['reason'] = 'UNKNOWN_FRONT'
            return result
        result['pose'] = list(pose)
        # Progress strictly in order. No skipping ahead based on x alone.
        target = self.route['waypoints'][self.index]
        distance = math.hypot(target[0]-pose[0], target[1]-pose[1])
        if distance <= WAYPOINT_TOLERANCE_M:
            self.index += 1
            if self.index == len(self.route['waypoints']):
                self.arrived = True
                result.update(state='ARRIVED', reason='WAYPOINTS_COMPLETE',
                              waypoint_index=self.index, distance_m=0.0)
                return result
            target = self.route['waypoints'][self.index]
            distance = math.hypot(target[0]-pose[0], target[1]-pose[1])
        heading = wrap_angle(math.atan2(target[1]-pose[1], target[0]-pose[0])-pose[2])
        result.update(waypoint_index=self.index, target=list(target),
                      distance_m=round(distance, 3), heading_error_rad=round(heading, 4))
        # A short-range obstacle still blocks the *suggestion*; never a motor command.
        if front_m <= 1.20:
            result.update(state='HOLD', reason='OBSTACLE_TOO_CLOSE')
        elif abs(heading) > HEADING_TOLERANCE_RAD:
            result.update(state='PLAN', reason='HEADING_ALIGNMENT',
                          suggested_action='TURN_LEFT' if heading > 0 else 'TURN_RIGHT')
        else:
            result.update(state='PLAN', reason='SEGMENT_CLEARANCE_TO_VERIFY',
                          suggested_action='ADVANCE_CANDIDATE')
        return result


def load_world(path):
    return json.loads(Path(path).read_text())


def main():
    parser = argparse.ArgumentParser(description='PULSE-1 S1 waypoint static planner, NO MOTION')
    parser.add_argument('--world', type=Path, default=Path(__file__).resolve().parent.parent / 'runtime/pulse1_world.json')
    parser.add_argument('--side', choices=('LEFT', 'RIGHT'), default='LEFT')
    args = parser.parse_args()
    route = build_route(load_world(args.world), args.side)
    print(json.dumps(route, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
