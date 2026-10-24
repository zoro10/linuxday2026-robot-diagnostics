#!/usr/bin/env python3
"""Isolated behavioral checks of Supervisor without ROS/Gazebo access."""
import ast
import json
import os
import time
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class FakeFuture:
    def done(self): return True
    def result(self): return types.SimpleNamespace(success=True)

class FakeClient:
    def __init__(self, trace, available):
        self.trace, self.available = trace, available
        self.pause_requests = []
    def wait_for_service(self, timeout_sec=None):
        self.trace.append(('service_wait', timeout_sec))
        return self.available
    def service_is_ready(self): return self.available
    def call_async(self, req):
        self.pause_requests.append(req)
        self.trace.append(('pause_request', req.world_control.pause))
        return FakeFuture()

class FakePublisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(json.loads(msg.data))

class FakeNode:
    service_available = True
    def __init__(self, name):
        self.trace = [('node', name)]
        self.pub_test = FakePublisher()
    def create_client(self, _, topic):
        self.trace.append(('client', topic))
        self.client_test = FakeClient(self.trace, self.service_available)
        return self.client_test
    def create_subscription(self, *args): return object()
    def create_publisher(self, *args): return self.pub_test
    def create_timer(self, period, callback):
        self.trace.append(('timer', period))
        return object()

class String:
    data = None

class ControlWorld:
    class Request:
        def __init__(self): self.world_control = types.SimpleNamespace(pause=None, multi_step=None)

class SupervisorStartupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        src = (ROOT / 'scripts/pulse1_v6_supervisor.py').read_text()
        mod = ast.parse(src)
        sup_class = next(node for node in mod.body if isinstance(node, ast.ClassDef) and node.name == 'Supervisor')
        isolated = ast.Module(body=[sup_class], type_ignores=[])
        ast.fix_missing_locations(isolated)
        ns = {'Node': FakeNode, 'String': String, 'ControlWorld': ControlWorld,
              'time': time, 'json': json,
              'health': lambda status, age: 'HEALTHY' if status and status.get('ok') else 'GUARD_LOST'}
        exec(compile(isolated, '<supervisor-isolation>', 'exec'), ns)
        cls.Supervisor = ns['Supervisor']

    def test_service_discovered_before_watchdog_timer(self):
        s = self.Supervisor(True)
        self.assertEqual([x[0] for x in s.trace].index('service_wait') + 1,
                         [x[0] for x in s.trace].index('timer'))
        self.assertTrue(s.client_test.available)
        self.assertFalse(s.pause_ack)
        self.assertIsNone(s.pause_error)

    def test_fail_closed_if_pause_service_missing(self):
        previous = FakeNode.service_available
        try:
            FakeNode.service_available = False
            with self.assertRaisesRegex(RuntimeError, 'SUPERVISOR_PAUSE_SERVICE_DISCOVERY_TIMEOUT'):
                self.Supervisor(True)
        finally:
            FakeNode.service_available = previous

    def test_disarmed_does_not_wait_for_pause_service(self):
        previous = FakeNode.service_available
        try:
            FakeNode.service_available = False
            s = self.Supervisor(False)
            self.assertNotIn('service_wait', [x[0] for x in s.trace])
            self.assertIn('timer', [x[0] for x in s.trace])
        finally:
            FakeNode.service_available = previous

    def test_fresh_healthy_guard_does_not_raise_emergency(self):
        s = self.Supervisor(True)
        s.status = {'ok': True}
        s.last = time.monotonic()
        s.tick()
        self.assertEqual(s.pub_test.messages[-1]['state'], 'HEALTHY')
        self.assertEqual(len(s.client_test.pause_requests), 0)

    def test_unsafe_guard_triggers_pause_and_emergency_latch(self):
        s = self.Supervisor(True)
        s.started -= 3
        s.tick()  # expired start phase, Guard never received
        self.assertTrue(s.pause_ack)
        self.assertEqual(len(s.client_test.pause_requests), 1)
        self.assertEqual(s.pub_test.messages[-1]['state'], 'EMERGENCY_PAUSE')
        s.status = {'ok': True}
        s.last = time.monotonic()
        s.tick()
        self.assertEqual(s.pub_test.messages[-1]['state'], 'EMERGENCY_PAUSE')
        self.assertEqual(len(s.client_test.pause_requests), 1)

if __name__ == '__main__':
    unittest.main(verbosity=2)
