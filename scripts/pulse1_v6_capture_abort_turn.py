#!/usr/bin/env python3
"""Read-only audit of latest partial TURN abort; NO recovery or movement."""
import json
import time
from pathlib import Path
import rclpy
from pulse1_v6_sequencer import Monitor, gazebo_pose, ROOT, CMD, REQUEST
from pulse1_v6_abort_resume_policy import capture_turn_abort, _aligned


def main():
    rclpy.init(args=[])
    node = Monitor(active=False)
    try:
        deadline = time.monotonic()+10
        fields = ('paused', 'guard', 'gf', 'sup', 'cmd')
        while time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if all(node.fresh(k,.7) for k in fields):
                break
        if not all(node.fresh(k,.7) for k in fields):
            raise RuntimeError('TURN_ABORT_AUDIT_TELEMETRY_STALE')
        motors = node.get_publishers_info_by_topic(CMD)
        if not (len(motors)==1 and motors[0].node_name=='pulse1_v6_motor_guard'
                and len(node.get_publishers_info_by_topic(REQUEST))==0):
            raise RuntimeError('TURN_ABORT_AUDIT_ROS_GRAPH_DENIED')
        first = gazebo_pose()
        second = gazebo_pose()
        if not _aligned(first,second,.002,.005):
            raise RuntimeError('TURN_ABORT_GAZEBO_MOVED')
        # A Gazebo Transport subprocess blocks this ROS executor. Get NEW
        # fresh callbacks after returning; never relax freshness limits.
        since=time.monotonic()
        end=time.monotonic()+6
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if (node.seen.get('paused',0)>since
                    and all(node.fresh(k,.7) for k in fields)):
                break
        if not (node.seen.get('paused',0)>since
                and all(node.fresh(k,.7) for k in fields)):
            raise RuntimeError('TURN_ABORT_AUDIT_TELEMETRY_LOST')
        evidence = capture_turn_abort(
            node.value('guard'),node.value('gf'),node.value('sup'),
            node.data.get('paused'),node.data.get('cmd'),second)
        dest_dir=ROOT/'runtime'/'evidence'
        dest_dir.mkdir(parents=True,exist_ok=True)
        dest=dest_dir/('v6_abort_turn_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'.json')
        with dest.open('x') as file:
            json.dump(evidence,file,indent=2,sort_keys=True)
            file.write('\n')
        print('TURN ABORT EVIDENCE:',dest)
        print('POSE LOCKED:',tuple(evidence['gz_pose']))
        print('PARTIAL MOTION RECORDED / NO REARM / NO MOTION')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__': main()
