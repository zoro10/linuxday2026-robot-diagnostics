#!/usr/bin/env python3
"""Read-only capture of the WP2 odometry ABORT before restarting any nodes."""
import json
import time
import rclpy
from pulse1_v6_sequencer import Monitor, gazebo_pose, ROOT, CMD, REQUEST
from pulse1_v6_abort_resume_policy import capture_wp2_odom_abort, _aligned


def main():
    rclpy.init(args=[])
    node = Monitor(active=False)
    try:
        fields = ('paused', 'guard', 'gf', 'sup', 'cmd')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(node.fresh(k, .7) for k in fields):
                break
        if not all(node.fresh(k, .7) for k in fields):
            raise RuntimeError('WP2_ABORT_AUDIT_TELEMETRY_STALE')
        motors = node.get_publishers_info_by_topic(CMD)
        if not (len(motors) == 1
                and motors[0].node_name == 'pulse1_v6_motor_guard'
                and len(node.get_publishers_info_by_topic(REQUEST)) == 0):
            raise RuntimeError('WP2_ABORT_AUDIT_ROS_GRAPH_DENIED')
        first = gazebo_pose()
        second = gazebo_pose()
        if not _aligned(first, second, .002, .005):
            raise RuntimeError('WP2_ABORT_AUDIT_GAZEBO_MOVED')
        # External Gazebo calls are blocking. Acquire new ROS status afterward.
        since = time.monotonic()
        end = time.monotonic() + 6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
            if (node.seen.get('paused', 0.0) > since
                    and all(node.fresh(k, .7) for k in fields)):
                break
        if not (node.seen.get('paused', 0.0) > since
                and all(node.fresh(k, .7) for k in fields)):
            raise RuntimeError('WP2_ABORT_AUDIT_TELEMETRY_LOST')
        evidence = capture_wp2_odom_abort(
            node.value('guard'), node.value('gf'), node.value('sup'),
            node.data.get('paused'), node.data.get('cmd'), second)
        path = ROOT/'runtime'/'evidence'/(
            'v6_abort_wp2_odom_'+time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())+'.json')
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('x') as stream:
            json.dump(evidence, stream, indent=2, sort_keys=True)
            stream.write('\n')
        print('WP2_ABORT_AUDIT_OK:', path)
        print('WP2_ABORT_POSE_LOCKED:', tuple(evidence['gz_pose']))
        print('ABORT_NOT_NOMINAL / NO REARM / NO MOTION')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
