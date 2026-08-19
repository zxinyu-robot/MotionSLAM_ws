#!/usr/bin/env bash
# 纯 Nav2 栈运行期间导出 topic/node/TF/参数快照
# 用法: ./scripts/capture_nav_snapshot.sh [export/evidence_YYYYMMDD/snapshots]
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS_DIR="$(cd "$SCRIPTS/.." && pwd)"
OUT="${1:-${WS_DIR}/export/snapshots_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${OUT}"

# 容器内路径（/ws 挂载宿主机 MotionSLAM_ws）
OUT_IN="/ws${OUT#${WS_DIR}}"

if ! docker ps --format '{{.Names}}' | grep -qx motionslam; then
  echo "ERROR: 容器 motionslam 未运行" >&2
  exit 1
fi

echo "[snapshot] 输出: ${OUT} (容器内 ${OUT_IN})"
echo "[snapshot] 需在 motionslam 容器内 Nav2 栈已启动"

docker exec motionslam bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  mkdir -p '${OUT_IN}'
  ros2 topic list > '${OUT_IN}/topic_list.txt'
  ros2 node list > '${OUT_IN}/node_list.txt'
  : > '${OUT_IN}/topic_hz.txt'
  for t in /lio/odom /lio/robo/odom /lio/cloud_world /cmd_vel /map /plan /local_costmap/costmap; do
    echo \"=== ros2 topic hz \${t} (5s) ===\" >> '${OUT_IN}/topic_hz.txt'
    timeout 6 ros2 topic hz \"\${t}\" 2>&1 >> '${OUT_IN}/topic_hz.txt' || true
  done
  ros2 param dump /controller_server > '${OUT_IN}/param_controller_server.yaml' 2>/dev/null || true
  ros2 param dump /planner_server > '${OUT_IN}/param_planner_server.yaml' 2>/dev/null || true
  ros2 param dump /local_costmap/local_costmap > '${OUT_IN}/param_local_costmap.yaml' 2>/dev/null || true
  ros2 param dump /global_costmap/global_costmap > '${OUT_IN}/param_global_costmap.yaml' 2>/dev/null || true
  cd /tmp
  timeout 15 ros2 run tf2_tools view_frames 2>/dev/null || true
  [[ -f /tmp/frames.pdf ]] && cp /tmp/frames.pdf /tmp/frames.gv '${OUT_IN}/' 2>/dev/null || true
  ros2 param get /cmd_vel_forwarder backend > '${OUT_IN}/param_forwarder_backend.txt' 2>/dev/null || true
"

cat > "${OUT}/TF_CHAIN.txt" <<'EOF'
主链（纯 Nav2 验收口径，以 view_frames 输出为准核对）:
  world → base_link（Super-LIO/ReLoc 发布）
  map → odom → base_link（Nav2 标准链，若启用）
请打开 frames.pdf 确认实际 parent/child 与 timestamp。
EOF

echo "[snapshot] 完成: ${OUT}"
ls -la "${OUT}"
