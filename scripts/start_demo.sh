#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="pulse1"

set +u
source /opt/ros/jazzy/setup.bash
set -u

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Demo gia' attiva nella sessione $SESSION"
    exit 1
fi

if pgrep -af 'gz sim' >/dev/null; then
    echo "ATTENZIONE: Gazebo e' gia' in esecuzione."
    echo "Ferma prima la simulazione precedente."
    exit 1
fi

trap 'echo "Avvio fallito: arresto la sessione"; tmux kill-session -t "$SESSION" 2>/dev/null || true' ERR

echo "[1/4] Avvio Gazebo..."
tmux new-session -d -s "$SESSION" -n gazebo \
  "bash -c 'source /opt/ros/jazzy/setup.bash; exec gz sim -s -r --headless-rendering \"$ROOT/simulation/worlds/pulse1.sdf\" -v 3'"

echo "[2/4] Attendo il mondo..."
ready=0
for i in {1..30}; do
    if gz service -l 2>/dev/null | grep -q '/world/empty/create'; then
        ready=1
        break
    fi
    sleep 1
done

if [[ "$ready" -ne 1 ]]; then
    echo "Gazebo non pronto."
    tmux kill-session -t "$SESSION"
    exit 1
fi

ros2 run ros_gz_sim create \
  --world empty \
  --file "$ROOT/simulation/models/pulse1/model.sdf" \
  --name pulse1_proto \
  -x 0 -y 0 -z 0.5

echo "[3/4] Avvio bridge ROS/Gazebo..."
tmux new-window -t "$SESSION" -n bridge \
  "bash -c 'source /opt/ros/jazzy/setup.bash; exec ros2 run ros_gz_bridge parameter_bridge \"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock\" \"/model/pulse1_proto/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist\" \"/model/pulse1_proto/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry\" \"/pulse1/imu@sensor_msgs/msg/Imu[gz.msgs.IMU\" \"/pulse1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan\" --ros-args -r __node:=pulse1_bridge'"

echo "[4/4] Avvio controller..."
tmux new-window -t "$SESSION" -n controller \
  "bash -c 'source /opt/ros/jazzy/setup.bash; cd \"$ROOT\"; exec python3 scripts/pulse1_controller.py'"

trap - ERR
echo "Demo avviata. Sessione tmux: $SESSION"
tmux list-windows -t "$SESSION"
