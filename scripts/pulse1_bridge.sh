#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

case "${1:-}" in
  main)
    exec ros2 run ros_gz_bridge parameter_bridge \
      '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
      '/model/pulse1_proto/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
      '/model/pulse1_proto/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
      '/pulse1/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
      --ros-args -r __node:=pulse1_bridge
    ;;
  stats)
    exec ros2 run ros_gz_bridge parameter_bridge \
      '/world/empty/stats@ros_gz_interfaces/msg/WorldStatistics[gz.msgs.WorldStatistics' \
      --ros-args -r __node:=pulse1_stats_bridge
    ;;
  lidar)
    exec ros2 run ros_gz_bridge parameter_bridge \
      '/pulse1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
      --ros-args -r __node:=pulse1_lidar_bridge
    ;;
  rear)
    exec ros2 run ros_gz_bridge parameter_bridge \
      '/pulse1/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
      --ros-args -r __node:=pulse1_lidar_rear_bridge
    ;;
  *)
    echo "Uso: $0 {main|lidar|rear}" >&2
    exit 2
    ;;
esac
