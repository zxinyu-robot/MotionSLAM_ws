#!/usr/bin/env bash
# 建图位姿图后端联调：关键帧计数、回环事件、map→world 校正（在容器内执行）
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
fi
if [[ -f /ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

echo "=== 期望已启动: mapping_walk / slam_viz / mapping_stack (含 pose_graph_backend) ==="
echo "运动: 线速 < 0.35 m/s，角速 < 0.45 rad/s (mapping_motion_guard)"
echo ""

missing=0
for t in /lio/backend/keyframe_count /lio/backend/loop_closed /lio/map_to_odom; do
  if ! ros2 topic list 2>/dev/null | grep -qx "$t"; then
    echo "[WARN] 话题不存在: $t"
    missing=$((missing + 1))
  fi
done
if [[ $missing -gt 0 ]]; then
  echo "请先: ros2 launch motionslam_bringup mapping_walk.launch.py"
  echo "  或: ros2 launch motionslam_bringup slam_viz.launch.py"
  exit 1
fi

echo "=== 当前关键帧数 ==="
ros2 topic echo /lio/backend/keyframe_count --once

echo ""
echo "=== 持续监控 (每 2s 刷新); Ctrl+C 结束 ==="
echo "另开终端可看: ros2 run tf2_ros tf2_echo map world"
echo "回环日志: pose_graph_backend 节点 stdout 含 'loop closed i=… j=…'"
echo ""

prev_count=""
while true; do
  count="$(ros2 topic echo /lio/backend/keyframe_count --once 2>/dev/null | awk '/data:/ {print $2; exit}')"
  motion_ok="$(ros2 topic echo /lio/mapping_motion_ok --once 2>/dev/null | awk '/data:/ {print $2; exit}')"
  ts="$(date +%H:%M:%S)"
  if [[ -n "$count" && "$count" != "$prev_count" ]]; then
    echo "[$ts] keyframes=$count motion_ok=${motion_ok:-?}"
    prev_count="$count"
  fi
  sleep 2
done
