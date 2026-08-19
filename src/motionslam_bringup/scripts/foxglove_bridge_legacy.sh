#!/usr/bin/env bash
# 启动 legacy foxglove_bridge 0.8.2 (foxglove.websocket.v1, 端口 8766)
set -euo pipefail

source /opt/ros/humble/setup.bash
if [[ ! -f /opt/foxglove_legacy/setup.bash ]]; then
  echo "ERROR: /opt/foxglove_legacy 未安装, 请重建 Docker 镜像" >&2
  exit 1
fi
source /opt/foxglove_legacy/setup.bash
[[ -f /ws/install/setup.bash ]] && source /ws/install/setup.bash

PREFIX="$(ros2 pkg prefix motionslam_bringup 2>/dev/null || true)"
PARAMS="${PREFIX}/share/motionslam_bringup/config/foxglove_legacy.yaml"
if [[ ! -f "${PARAMS}" ]]; then
  PARAMS="/ws/src/motionslam_bringup/config/foxglove_legacy.yaml"
fi

exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  --params-file "${PARAMS}" \
  -r __node:=foxglove_bridge_legacy
