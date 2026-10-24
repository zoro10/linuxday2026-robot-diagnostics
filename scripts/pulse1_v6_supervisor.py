#!/usr/bin/env python3
"""Independent ROS supervisor for exclusive V6 guard heartbeat.

No command velocity publisher; when armed, triggers PAUSE on guard loss/abort.
"""
import argparse
import json
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros_gz_interfaces.srv import ControlWorld


from pulse1_v6_supervisor_policy import health


class Supervisor(Node):
    def __init__(self, armed):
        super().__init__('pulse1_v6_supervisor')
        self.armed = armed
        self.started = time.monotonic()
        self.last = None
        self.status = None
        self.future = None
        self.pause_error = None
        self.pause_ack = False
        self.client = self.create_client(ControlWorld, '/world/empty/control')
        self.sub = self.create_subscription(String, '/pulse1/v6_motor_guard', self.on_guard, 10)
        self.pub = self.create_publisher(String, '/pulse1/v6_supervisor_status', 10)
        # Startup-only discovery barrier. The current session may be restarted
        # with Gazebo already PAUSED; do not arm a watchdog timer while this
        # node cannot yet discover the independent emergency PAUSE service.
        # A discovery failure is fail-closed: no HEALTHY heartbeat is published.
        if self.armed and not self.client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('SUPERVISOR_PAUSE_SERVICE_DISCOVERY_TIMEOUT')
        # The existing 1.5 s Guard-discovery window begins only after PAUSE
        # service discovery. Runtime 0.8 s watchdog and latched faults are intact.
        self.started = time.monotonic()
        self.create_timer(.1, self.tick)

    def on_guard(self, m):
        try:
            d = json.loads(m.data)
            self.status = d if isinstance(d, dict) else None
        except (ValueError, TypeError):
            self.status = None
        self.last = time.monotonic()

    def tick(self):
        now = time.monotonic()
        age = now-self.last if self.last is not None else None
        state = health(self.status, age)
        if self.last is None and now-self.started <= 1.5:
            state = 'STARTUP'
        if self.armed and state not in ('HEALTHY', 'STARTUP') and self.future is None and not self.pause_ack and not self.pause_error:
            if not self.client.service_is_ready():
                self.pause_error = 'PAUSE_SERVICE_NOT_READY'
            else:
                req = ControlWorld.Request()
                req.world_control.pause = True
                req.world_control.multi_step = 0
                self.future = self.client.call_async(req)
                self.sent = now
        if self.future is not None:
            if self.future.done():
                try:
                    ans = self.future.result()
                    self.pause_ack = ans is not None and ans.success is True
                except Exception:
                    self.pause_ack = False
                if not self.pause_ack:
                    self.pause_error = 'PAUSE_FAILED'
                self.future = None
            elif now-self.sent > 3:
                self.future = None
                self.pause_error = 'PAUSE_TIMEOUT'
        data = dict(source='v6_supervisor', armed=self.armed,
                    state=state if not (self.pause_error or self.pause_ack) else 'EMERGENCY_PAUSE',
                    pause_ack=self.pause_ack, pause_error=self.pause_error,
                    motor_commands=False)
        msg=String();msg.data=json.dumps(data)
        self.pub.publish(msg)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--armed', action='store_true')
    a=p.parse_args()
    if a.armed and os.environ.get('PULSE1_V6_SUPERVISOR_ARM') != 'YES':
        raise SystemExit('Required PULSE1_V6_SUPERVISOR_ARM=YES')
    rclpy.init(args=[])
    node=None
    try:
        node=Supervisor(a.armed)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
