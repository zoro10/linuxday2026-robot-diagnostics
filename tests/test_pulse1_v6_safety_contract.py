import copy
import math
import unittest

from pulse1_v6_safety_contract import (
    V6GeofenceSegment, valid_request, v6_guard_velocity,
    geofence_permission, stopped_independently,
)


def request():
    return dict(source='v6_segment_executor', mode='AVOID', motor_commands=False,
                phase='DRIVE', segment_id='segment-0001', waypoint_index=0,
                kind='ADVANCE', start_pose=[0., 0., 0.], target=[2.368566, 0.],
                limit=.2, direction='', proposed_linear_m_s=.1,
                proposed_angular_rad_s=0., odom_ok=True, lidar_ok=True,
                rear_lidar_ok=True, imu_ok=True)


def monitor(r=None):
    r = r or request()
    return dict(source='v6_geofence_watchdog', mode='AVOID', armed=True,
                ready_for_motion=True, action='MONITOR',
                reason='WITHIN_SEGMENT_ENVELOPE', motor_commands=False,
                segment_id=r['segment_id'], kind=r['kind'],
                waypoint_index=r['waypoint_index'])


def observer():
    sectors = {key: {'distance_m': 4.2, 'clear': False}
               for key in ('front', 'left', 'right')}
    readings = {'front': {'distance_m': 4.2, 'clear': False, 'sectors': copy.deepcopy(sectors)},
                'rear': {'distance_m': 4.2, 'clear': False, 'sectors': copy.deepcopy(sectors)}}
    return dict(source='avoid_observer', mode='AVOID', state='SENSOR_READY',
                motor_commands=False, odom_ok=True, lidar_ok=True,
                rear_lidar_ok=True, imu_ok=True, readings=readings)


def blocked_guard():
    return dict(source='motor_guard_live', mode='AVOID', motor_commands=True,
                guard_state='BLOCKED', permitted_linear_m_s=0.,
                permitted_angular_rad_s=0.)


def samples(**extra):
    d = dict(request=request(), request_age_s=.03, pose=(0., 0., 0.),
             pose_age_s=.03, now_s=100., observer=observer(), observer_age_s=.03,
             guard=blocked_guard(), guard_age_s=.03, cmd=(0., 0.), cmd_age_s=.03,
             world_paused=True, world_age_s=.03, pause_ack=False)
    d.update(extra)
    return d


class TestV6Guard(unittest.TestCase):
    def allowed(self, r=None, wd=None, **changes):
        r = r or request()
        d = dict(request_age_s=.03, watchdog=wd or monitor(r), watchdog_age_s=.03,
                 observer=observer(), observer_age_s=.03, odom_age_s=.03)
        d.update(changes)
        return v6_guard_velocity(r, **d)

    def test_malformed_request_fails_closed(self):
        for candidate in (None, 1, [], {'segment_id': None},
                          {'segment_id': 10, 'waypoint_index': None}):
            self.assertFalse(valid_request(candidate))
            self.assertEqual(v6_guard_velocity(candidate, request_age_s=.03,
                             watchdog=None, watchdog_age_s=.03, observer=None,
                             observer_age_s=.03, odom_age_s=.03), (0., 0.))

    def test_none_watchdog_fails_closed(self):
        self.assertFalse(geofence_permission(None, None, None))
        self.assertEqual(self.allowed(watchdog='notadict'), (0.,0.))

    def test_advance_works_with_independent_permission(self):
        self.assertEqual(self.allowed(), (.1, 0.))

    def test_turn_left_with_independent_permission(self):
        r=request();r.update(kind='TURN', direction='LEFT', limit=.2,
                             proposed_linear_m_s=0., proposed_angular_rad_s=.2)
        self.assertEqual(self.allowed(r), (0., .2))

    def test_turn_right(self):
        r=request();r.update(kind='TURN', direction='RIGHT', limit=.2,
                             proposed_linear_m_s=0., proposed_angular_rad_s=-.2)
        self.assertEqual(self.allowed(r), (0., -.2))

    def test_turn_direction_mismatch(self):
        r=request();r.update(kind='TURN', direction='RIGHT', limit=.2,
                             proposed_linear_m_s=0., proposed_angular_rad_s=.2)
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_turn_limit_rejected(self):
        r=request();r.update(kind='TURN', direction='LEFT', limit=.3,
                             proposed_linear_m_s=0., proposed_angular_rad_s=.2)
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_translation_limit_rejected(self):
        r=request();r['limit']=.201
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_reverse_denied(self):
        r=request();r['proposed_linear_m_s']=-.1
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_linear_angular_combination_denied(self):
        r=request();r['proposed_angular_rad_s']=.05
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_stale_request_denied(self):
        self.assertEqual(self.allowed(request_age_s=.4), (0., 0.))

    def test_stale_watchdog_denied(self):
        self.assertEqual(self.allowed(watchdog_age_s=.3), (0., 0.))

    def test_stale_odom_denied(self):
        self.assertEqual(self.allowed(odom_age_s=.61), (0., 0.))

    def test_stale_observer_denied(self):
        self.assertEqual(self.allowed(observer_age_s=.61), (0., 0.))

    def test_obstacle_front_denied(self):
        o=observer();o['readings']['front']['distance_m']=1.0
        self.assertEqual(self.allowed(observer=o), (0.,0.))

    def test_unknown_front_denied(self):
        o=observer();o['readings']['front']=None
        self.assertEqual(self.allowed(observer=o), (0.,0.))

    def test_unknown_turn_sector_denied(self):
        r=request();r.update(kind='TURN', direction='LEFT', limit=.2,
                             proposed_linear_m_s=0., proposed_angular_rad_s=.2)
        o=observer();o['readings']['rear']['sectors']['right']=None
        self.assertEqual(self.allowed(r, observer=o), (0.,0.))

    def test_unhealthy_observer_denied(self):
        o=observer();o['imu_ok']=False
        self.assertEqual(self.allowed(observer=o), (0., 0.))

    def test_watchdog_wrong_segment_denied(self):
        w=monitor();w['segment_id']='segment-OTHER'
        self.assertEqual(self.allowed(wd=w), (0., 0.))

    def test_watchdog_unarmed_denied(self):
        w=monitor();w['armed']=False
        self.assertEqual(self.allowed(wd=w), (0., 0.))

    def test_watchdog_wrong_kind_denied(self):
        w=monitor();w['kind']='TURN'
        self.assertEqual(self.allowed(wd=w), (0., 0.))

    def test_watchdog_paused_denied(self):
        w=monitor();w['action']='PAUSE'
        self.assertEqual(self.allowed(wd=w), (0., 0.))

    def test_nan_speed_denied(self):
        r=request();r['proposed_linear_m_s']=math.nan
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_legacy_v5_denied(self):
        r=request();r['source']='avoid_navigator'
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_bool_speed_denied(self):
        r=request();r['proposed_linear_m_s']=True
        self.assertEqual(self.allowed(r), (0., 0.))

    def test_missing_sensor_flag_denied(self):
        r=request();del r['imu_ok']
        self.assertEqual(self.allowed(r), (0., 0.))


class TestV6Watchdog(unittest.TestCase):
    def test_start_requires_paused_zero(self):
        w=V6GeofenceSegment()
        v=w.tick(**samples(cmd=(.02,0.)))
        self.assertEqual(v['action'], 'PAUSE')
        self.assertEqual(v['reason'], 'START_NOT_STOPPED')

    def test_start_with_zero_reports_non_armable_monitor(self):
        w=V6GeofenceSegment()
        v=w.tick(**samples())
        self.assertEqual(v['action'], 'MONITOR')
        self.assertFalse(v['armed'])
        self.assertFalse(v['ready_for_motion'])
        self.assertTrue(v['candidate_for_motion'])

    def test_no_request_waits(self):
        w=V6GeofenceSegment()
        self.assertEqual(w.tick(**samples(request=None))['action'], 'WAIT')

    def test_invalid_initial_pose_aborts(self):
        w=V6GeofenceSegment()
        self.assertEqual(w.tick(**samples(pose=(.3,0,0)))['reason'], 'START_POSE_MISMATCH')

    def test_advance_and_stop_ack(self):
        w=V6GeofenceSegment()
        self.assertEqual(w.tick(**samples())['action'], 'MONITOR')
        self.assertEqual(w.tick(**samples(pose=(.12,0,0), now_s=102.))['action'], 'MONITOR')
        self.assertEqual(w.tick(**samples(pose=(.2,0,0), now_s=103.))['action'], 'STOPPING')
        r=request();r.update(phase='HOLD', reason='SEGMENT_TARGET_REACHED',
                             proposed_linear_m_s=0.)
        self.assertEqual(w.tick(**samples(request=r, pose=(.2,0,0), now_s=103.5,
                                           pause_ack=True))['action'], 'COMPLETE')
        self.assertEqual(w.tick(**samples(request=request(), now_s=104.))['action'], 'COMPLETE')

    def test_early_hold_aborts(self):
        w=V6GeofenceSegment();w.tick(**samples())
        r=request();r.update(phase='HOLD', reason='SEGMENT_TARGET_REACHED',
                             proposed_linear_m_s=0.)
        self.assertEqual(w.tick(**samples(request=r, pose=(.05,0,0), now_s=101.))['reason'],
                         'EARLY_OR_BAD_COMPLETION')

    def test_lateral_breach_latches(self):
        w=V6GeofenceSegment();w.tick(**samples())
        self.assertEqual(w.tick(**samples(pose=(.05,.04,0), now_s=101.))['action'], 'PAUSE')
        self.assertEqual(w.tick(**samples(pose=(0,0,0), now_s=102.))['action'], 'PAUSE')

    def test_overshoot_aborts(self):
        w=V6GeofenceSegment();w.tick(**samples())
        self.assertEqual(w.tick(**samples(pose=(.22,0,0), now_s=101.))['reason'],
                         'SEGMENT_ENVELOPE_BREACHED')

    def test_id_swap_aborts(self):
        w=V6GeofenceSegment();w.tick(**samples())
        r=request();r['segment_id']='segment-0002'
        self.assertEqual(w.tick(**samples(request=r, now_s=101.))['action'], 'PAUSE')

    def test_obstacle_appears_emergency_pause(self):
        w=V6GeofenceSegment();w.tick(**samples())
        o=observer();o['readings']['front']['distance_m']=.4
        self.assertEqual(w.tick(**samples(observer=o, now_s=101.))['reason'],
                         'OBSTACLE_CLEARANCE_LOST')

    def test_mutated_hold_aborts(self):
        w=V6GeofenceSegment();w.tick(**samples())
        r=request();r.update(phase='HOLD', reason='SEGMENT_TARGET_REACHED',
                             proposed_linear_m_s=0., target=[9.,9.])
        self.assertEqual(w.tick(**samples(request=r, pose=(.2,0,0), now_s=101.))['reason'],
                         'REQUEST_MUTATED')

    def test_stale_sensor_aborts(self):
        w=V6GeofenceSegment();w.tick(**samples())
        self.assertEqual(w.tick(**samples(observer_age_s=.9, now_s=101.))['reason'],
                         'OBSERVER_STALE')

    def test_timeout_aborts(self):
        w=V6GeofenceSegment();w.tick(**samples())
        self.assertEqual(w.tick(**samples(now_s=131.))['reason'], 'SEGMENT_TIMEOUT')

    def test_turn_envelope(self):
        w=V6GeofenceSegment()
        r=request();r.update(kind='TURN', limit=.2, direction='LEFT',
                             proposed_linear_m_s=0., proposed_angular_rad_s=.2)
        self.assertEqual(w.tick(**samples(request=r))['action'], 'MONITOR')
        self.assertEqual(w.tick(**samples(request=r, pose=(0,0,.2), now_s=101.))['action'],
                         'STOPPING')

    def test_wrong_direction_turn_aborts(self):
        w=V6GeofenceSegment()
        r=request();r.update(kind='TURN', limit=.2, direction='LEFT',
                             proposed_linear_m_s=0., proposed_angular_rad_s=.2)
        w.tick(**samples(request=r))
        self.assertEqual(w.tick(**samples(request=r, pose=(0,0,-.05), now_s=101.))['reason'],
                         'SEGMENT_ENVELOPE_BREACHED')

    def test_stop_timeout_aborts(self):
        w=V6GeofenceSegment();w.tick(**samples())
        w.tick(**samples(pose=(.2,0,0), now_s=101.))
        r=request();r.update(phase='HOLD', reason='SEGMENT_TARGET_REACHED',
                             proposed_linear_m_s=0.)
        self.assertEqual(w.tick(**samples(request=r, pose=(.2,0,0), now_s=104.5))['reason'],
                         'STOP_TIMEOUT')

    def test_stop_requires_pause_ack(self):
        w=V6GeofenceSegment();w.tick(**samples())
        w.tick(**samples(pose=(.2,0,0), now_s=101.))
        r=request();r.update(phase='HOLD', reason='SEGMENT_TARGET_REACHED',
                             proposed_linear_m_s=0.)
        self.assertEqual(w.tick(**samples(request=r, pose=(.2,0,0), now_s=102.,
                                           pause_ack=False))['action'], 'STOPPING')

    def test_non_zero_stop_cmd_denies_ack(self):
        w=V6GeofenceSegment();w.tick(**samples())
        w.tick(**samples(pose=(.2,0,0), now_s=101.))
        r=request();r.update(phase='HOLD', reason='SEGMENT_TARGET_REACHED',
                             proposed_linear_m_s=0.)
        self.assertEqual(w.tick(**samples(request=r, pose=(.2,0,0), now_s=102.,
                                           pause_ack=True,cmd=(.05,0.)))['action'], 'STOPPING')

    def test_missing_ack_never_completes(self):
        w=V6GeofenceSegment();w.tick(**samples())
        w.tick(**samples(pose=(.2,0,0), now_s=101.))
        r=request();r.update(phase='HOLD', reason='SEGMENT_TARGET_REACHED',
                             proposed_linear_m_s=0.)
        self.assertEqual(w.tick(**samples(request=r, pose=(.2,0,0), now_s=102.,
                                           pause_ack=False))['action'], 'STOPPING')


if __name__ == '__main__':
    unittest.main()
