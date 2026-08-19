#!/usr/bin/env bash
# 宿主机: 停官方 SLAM -> 进容器 -> 建图 + Foxglove
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

echo "[1/3] 停用 unitree_slam..."
"$SCRIPTS/map/stop_unitree_slam.sh"

echo "[2/3] 进入/启动容器 (若已运行则 exec)..."
if docker ps --format '{{.Names}}' | grep -qx motionslam; then
  EXEC="docker exec -it motionslam bash -lc"
else
  echo "请先运行: ./docker/run_container.sh"
  exit 1
fi

echo "[3/3] 容器内启动 slam_viz (规范: mapping_stack + Foxglove)..."
$EXEC 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch motionslam_bringup slam_viz.launch.py'
