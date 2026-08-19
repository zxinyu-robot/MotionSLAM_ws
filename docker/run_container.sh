#!/usr/bin/env bash
# 启动(或进入) MotionSLAM Humble 开发容器
# 用法: ./docker/run_container.sh [命令]   无参数则进入交互 bash
#
# 需在 MotionSLAM_ws 目录下执行:
#   cd ~/MotionSLAM_ws && ./docker/run_container.sh
set -euo pipefail

IMAGE=motionslam:humble
NAME=motionslam
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CMD="${*:-bash}"

DOCKER_IT=(-i)
if [[ -t 0 && -t 1 ]]; then
  DOCKER_IT=(-it)
fi

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"
}

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "${NAME}"
}

if container_running; then
  exec docker exec "${DOCKER_IT[@]}" "${NAME}" bash -c "${CMD}"
fi

if container_exists; then
  echo "[motionslam] 容器已存在但未运行, 正在启动 ${NAME} ..."
  if [[ $# -eq 0 ]]; then
    if [[ -t 0 && -t 1 ]]; then
      exec docker start -ai "${NAME}"
    fi
    docker start "${NAME}" >/dev/null
    exec docker exec "${DOCKER_IT[@]}" "${NAME}" bash
  fi
  docker start "${NAME}" >/dev/null
  exec docker exec "${DOCKER_IT[@]}" "${NAME}" bash -c "${CMD}"
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "ERROR: 镜像 ${IMAGE} 不存在, 请先构建: cd ${WS_DIR}/docker && docker build -t ${IMAGE} ." >&2
  exit 1
fi

echo "[motionslam] 创建新容器 ${NAME} ..."
# NOTE: --network host + --ipc host 让容器内 DDS 直通宿主机 eth0, 与狗域 0 互通
exec docker run "${DOCKER_IT[@]}" --rm \
  --name "${NAME}" \
  --network host \
  --ipc host \
  --pid host \
  -v "${WS_DIR}:/ws" \
  -v /dev:/dev \
  --privileged \
  "${IMAGE}" \
  bash -c "${CMD}"
