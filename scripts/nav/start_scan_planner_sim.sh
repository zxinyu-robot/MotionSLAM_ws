#!/usr/bin/env bash
# 容器内启动 SCAN-Planner 仿真（RViz 需另开终端）
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"

if ! docker ps --format '{{.Names}}' | grep -qx motionslam; then
  echo "请先运行: ./docker/run_container.sh"
  exit 1
fi

docker exec -it motionslam bash -lc "
  source /opt/ros/humble/setup.bash
  source /ws/scan_planner_ws/install/setup.bash
  ros2 launch scan_planner run.launch.py \
    is_real_world:=false navi_mode:=1 sensor_type:=lidar \
    controller_mode:=closed_loop use_gpu:=false
"
