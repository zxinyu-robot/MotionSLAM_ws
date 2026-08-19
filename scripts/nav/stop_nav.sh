#!/usr/bin/env bash
# 停止 Demo 导航栈 + 急停 + 恢复手柄（宿主机与容器内均可调用）
set -eo pipefail
set +u

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash && -f /ws/install/setup.bash ]]; then
  :
elif docker ps --format '{{.Names}}' 2>/dev/null | grep -qx motionslam; then
  exec docker exec motionslam bash -lc 'bash /ws/scripts/nav/stop_nav.sh'
else
  echo "ERROR: motionslam 容器未运行" >&2
  exit 1
fi

estop() {
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    [[ -f /ws/install/setup.bash ]] && source /ws/install/setup.bash
    [[ -f "${ROOT}/install/setup.bash" ]] && source "${ROOT}/install/setup.bash"
  fi
  if command -v ros2 >/dev/null 2>&1; then
    timeout 5 ros2 run motionslam_pipeline estop_go2 2>/dev/null || true
  fi
}

restore_remote() {
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    [[ -f /ws/install/setup.bash ]] && source /ws/install/setup.bash
    [[ -f "${ROOT}/install/setup.bash" ]] && source "${ROOT}/install/setup.bash"
  fi
  if command -v ros2 >/dev/null 2>&1; then
    timeout 8 ros2 run motionslam_pipeline restore_go2_remote 2>/dev/null || true
  fi
}

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  [[ -f /ws/install/setup.bash ]] && source /ws/install/setup.bash
fi

echo "[1/3] 取消 Demo 任务..."
if command -v ros2 >/dev/null 2>&1; then
  timeout 3 ros2 topic pub --once /demo/mission/cancel std_msgs/msg/String "{data: 'stop'}" \
    >/dev/null 2>&1 || true
fi

echo "[2/3] 急停..."
estop
sleep 0.3

echo "[3/3] 停止 Demo / 建图进程..."
for p in super_lio_node livox_ros_driver2_node cmd_vel_forwarder foxglove_bridge \
  foxglove_bridge_legacy mapping_motion_guard mapping_keyframe_manager \
  pose_graph_backend loop_detection_node local_map_node stance_gate_node glass_suspect_layer motor_monitor_node \
  scan_planner_node closed_loop_controller open_loop_controller demo_navigation_context \
  pct_resident_planner virtual_obstacle_merge bev_glass_layer; do
  pkill -x "$p" 2>/dev/null || true
done
pkill -f "demo_scan_stack.launch.py" 2>/dev/null || true
pkill -f "demo_scan_bt_orchestrator.py" 2>/dev/null || true
pkill -f "demo_scan_lifecycle_autostart.py" 2>/dev/null || true
pkill -f "pct_planner_node.py" 2>/dev/null || true
pkill -f "pct_planner.launch.py" 2>/dev/null || true
pkill -f "rgb_bev_node.py" 2>/dev/null || true
pkill -f "semantic_token_uplink_node.py" 2>/dev/null || true
pkill -f "mapping_stack.launch.py" 2>/dev/null || true
pkill -f "mapping_walk.launch.py" 2>/dev/null || true
pkill -f "slam_viz.launch.py" 2>/dev/null || true
pkill -f "ros2 launch motionslam_bringup" 2>/dev/null || true
pkill -f "robot_pose_viz.py" 2>/dev/null || true
pkill -f "foxglove_bridge_legacy.sh" 2>/dev/null || true
sleep 0.5

for exe in cmd_vel_forwarder livox_ros_driver2_node super_lio_node \
  mapping_motion_guard mapping_keyframe_manager pose_graph_backend \
  loop_detection_node local_map_node stance_gate_node glass_suspect_layer motor_monitor_node \
  foxglove_bridge scan_planner_node closed_loop_controller open_loop_controller \
  demo_navigation_context pct_resident_planner virtual_obstacle_merge bev_glass_layer; do
  pkill -9 -f "/${exe}$" 2>/dev/null || true
  pkill -9 -f "lib/[^/]*/${exe}" 2>/dev/null || true
done

rm -f /dev/shm/motionslam_map 2>/dev/null || true

estop
sleep 0.5
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop 2>/dev/null || true
  ros2 daemon start 2>/dev/null || true
fi

restore_remote
echo "OK: Demo 导航栈已停止 (已急停, 手柄已恢复)"
