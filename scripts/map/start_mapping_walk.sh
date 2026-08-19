#!/usr/bin/env bash
# 宿主机: 建图试走栈 (L1 避障 + 位姿图后端 + Foxglove)
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

"$SCRIPTS/map/stop_unitree_slam.sh"

if ! docker ps --format '{{.Names}}' | grep -qx motionslam; then
  echo "请先: ./docker/run_container.sh"
  exit 1
fi

echo "容器内: mapping_walk.launch.py ..."
docker exec -it motionslam bash -lc \
  'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch motionslam_bringup mapping_walk.launch.py'
