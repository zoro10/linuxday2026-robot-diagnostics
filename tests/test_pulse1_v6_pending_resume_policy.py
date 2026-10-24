"""Offline fail-closed tests for the 2026-10-23 WP2 pre-motion recovery."""
import copy
import json
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from pulse1_v6_pending_resume_policy import (
    PENDING_GAZEBO, parse_gz_model_pose, verify_old_pending_audit,
    verify_pending_bootstrap,
)

AUDIT = pathlib.Path(__file__).resolve().parents[1] / 'runtime/evidence/v6_pending_audit_20261023T223055Z.json'


class PendingRecoveryPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.archive = json.loads(AUDIT.read_text())
        cls.timepoint = cls.archive['captured_at_epoch'] + 100.0

    def setUp(self):
        self.audit = copy.deepcopy(self.archive)
        self.guard = copy.deepcopy(self.audit['guard'])
        self.gf = copy.deepcopy(self.audit['geofence'])
        self.sup = copy.deepcopy(self.audit['supervisor'])
        self.guard['expected_next_segment_id'] = None
        self.guard['segment_id'] = None
        self.gf['reason'] = 'NOT_STARTED'
        self.gf['rearm_next_segment_id'] = None
        self.gf['rearm_previous_segment_id'] = None

    def verify(self, **kw):
        return verify_pending_bootstrap(
            kw.get('audit', self.audit), kw.get('guard', self.guard),
            kw.get('gf', self.gf), kw.get('sup', self.sup),
            kw.get('paused', True), kw.get('cmd', (0.0, 0.0)),
            kw.get('pose', PENDING_GAZEBO), now=kw.get('now', self.timepoint),
        )

    def test_valid_new_session(self):
        self.assertTrue(self.verify())

    def test_old_audit_is_not_nominal(self):
        self.assertFalse(self.audit['nominal_stop'])
        self.assertTrue(verify_old_pending_audit(self.audit, now=self.timepoint))

    def test_stale_audit_rejected(self):
        with self.assertRaisesRegex(ValueError, 'STALE'):
            self.verify(now=self.timepoint + 3600 * 7)

    def test_old_pending_id_invalid(self):
        a = copy.deepcopy(self.audit)
        a['guard']['expected_next_segment_id'] = 'different'
        with self.assertRaises(ValueError):
            self.verify(audit=a)

    def test_untrue_audit_flags_rejected(self):
        for flag in self.audit['checks']:
            with self.subTest(flag=flag):
                a = copy.deepcopy(self.audit)
                a['checks'][flag] = False
                with self.assertRaises(ValueError):
                    self.verify(audit=a)

    def test_old_nominal_forgery_rejected(self):
        a = copy.deepcopy(self.audit)
        a['nominal_stop'] = True
        with self.assertRaises(ValueError):
            self.verify(audit=a)

    def test_old_authorized_speed_rejected(self):
        a = copy.deepcopy(self.audit)
        a['guard']['permitted_linear_m_s'] = 0.1
        with self.assertRaises(ValueError):
            self.verify(audit=a)

    def test_pose_moved_rejected(self):
        for p in [(PENDING_GAZEBO[0]+0.030, *PENDING_GAZEBO[1:]),
                  (PENDING_GAZEBO[0],PENDING_GAZEBO[1],PENDING_GAZEBO[2]+0.04)]:
            with self.subTest(pose=p):
                with self.assertRaisesRegex(ValueError, 'POSE_CHANGED'):
                    self.verify(pose=p)

    def test_guards_cannot_preserve_old_id(self):
        for field in ('expected_next_segment_id','segment_id'):
            with self.subTest(field=field):
                g=copy.deepcopy(self.guard)
                g[field]='v6-route-5bd8fe220b5c73d92224'
                with self.assertRaises(ValueError):
                    self.verify(guard=g)

    def test_geofence_cannot_preserve_old_id(self):
        f=copy.deepcopy(self.gf)
        f['rearm_next_segment_id']='v6-route-5bd8fe220b5c73d92224'
        with self.assertRaises(ValueError):
            self.verify(gf=f)

    def test_wrong_geofence_stage(self):
        f=copy.deepcopy(self.gf)
        f['reason']='REARM_VERIFIED'
        with self.assertRaises(ValueError):
            self.verify(gf=f)

    def test_guard_emergency(self):
        g=copy.deepcopy(self.guard)
        g['emergency_latched']=True
        with self.assertRaises(ValueError):
            self.verify(guard=g)

    def test_guard_not_exclusive(self):
        g=copy.deepcopy(self.guard)
        g['cmd_vel_exclusive']=False
        with self.assertRaises(ValueError):
            self.verify(guard=g)

    def test_guard_forged_nominal(self):
        g=copy.deepcopy(self.guard)
        g['nominal_stop_latched']=True
        with self.assertRaises(ValueError):
            self.verify(guard=g)

    def test_supervisor_emergency(self):
        s=copy.deepcopy(self.sup)
        s['state']='EMERGENCY_PAUSE'
        with self.assertRaises(ValueError):
            self.verify(sup=s)

    def test_not_paused(self):
        with self.assertRaises(ValueError):
            self.verify(paused=False)

    def test_cmd_not_zero(self):
        with self.assertRaises(ValueError):
            self.verify(cmd=(0.01,0.0))

    def test_parse_topic_gazebo_model_origin(self):
        raw='''pose {
  name: "pulse1_proto"
  id: 33
  position {
    x: 2.9208203403625337
    y: 2.8213209207480694
    z: 0.32499990764664222
  }
  orientation {
    x: 1.1829125161176421e-09
    y: -7.4881606330280535e-08
    z: 0.015796648158089659
    w: 0.99987522516910277
  }
}
pose {
  name: "link"
  id: 5
  position {}
  orientation {w: 1}
}
'''
        pose=parse_gz_model_pose(raw)
        self.assertAlmostEqual(pose[0],PENDING_GAZEBO[0])
        self.assertAlmostEqual(pose[1],PENDING_GAZEBO[1])
        self.assertAlmostEqual(pose[2],PENDING_GAZEBO[2],places=5)

    def test_pose_output_missing(self):
        with self.assertRaises(ValueError):
            parse_gz_model_pose('pose { name: "other" }')

    def test_pose_topic_disagreement(self):
        base='''pose {\n  name: "pulse1_proto"\n  position {\n    x: %f\n    y: 2.82132\n  }\n  orientation {\n    w: 1\n  }\n}\n'''
        with self.assertRaisesRegex(ValueError, 'UNSTABLE'):
            parse_gz_model_pose(base % 2.92082 + base % 2.95082)


if __name__ == '__main__':
    unittest.main()
