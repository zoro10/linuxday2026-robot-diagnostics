#!/usr/bin/env python3
"""Read-only capture of WP1 abort evidence, before operator node restart."""
import json
import rclpy
import time
from pathlib import Path
from pulse1_v6_sequencer import Monitor, gazebo_pose, ROOT
from pulse1_v6_abort_resume_policy import capture_abort


def main():
    rclpy.init(args=[])
    node=Monitor(active=False)
    try:
        end=time.monotonic()+6
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)  # poll() rejects PAUSE
            if (node.fresh('guard',.6) and node.fresh('gf',.6)
                    and node.fresh('sup',.8) and node.fresh('paused',.8)
                    and node.fresh('cmd',.6)):
                break
        if not node.base_safe():
            raise RuntimeError('ABORT_AUDIT_BASE_NOT_SAFE')
        pose1=gazebo_pose()
        pose2=gazebo_pose()
        # Recheck all ROS preconditions after Gazebo transport operations.
        for _ in range(8): rclpy.spin_once(node,timeout_sec=.05)
        if not node.base_safe():
            raise RuntimeError('ABORT_AUDIT_SAFETY_CHANGED')
        from pulse1_v6_abort_resume_policy import _aligned
        if not _aligned(pose1,pose2,.002,.005):
            raise RuntimeError('ABORT_AUDIT_POSE_CHANGED')
        audit=capture_abort(node.value('guard'),node.value('gf'),
                            node.value('sup'),node.data.get('paused'),
                            node.data.get('cmd'),pose2)
        location=ROOT/'runtime'/'evidence'
        location.mkdir(parents=True,exist_ok=True)
        dest=location/('v6_abort_wp1_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'.json')
        if dest.exists():
            raise RuntimeError('ABORT_AUDIT_FILE_EXISTS')
        dest.write_text(json.dumps(audit,sort_keys=True,indent=2)+'\n')
        print('ABORT EVIDENCE:',dest)
        print('POSE LOCKED:',tuple(audit['gz_pose']))
        print('NO MOTION / NO REARM / ABORT NOT CLEARED')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__': main()
