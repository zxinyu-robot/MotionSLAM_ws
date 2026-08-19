#!/usr/bin/env bash
# 日常一条：确保容器 Up → 停官方 SLAM → 起 Demo 栈
# 入口: ./scripts/motionslam nav start
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
IMAGE="${MOTIONSLAM_IMAGE:-motionslam:humble}"
NAME="${MOTIONSLAM_NAME:-motionslam}"

WITH_FOXGLOVE="${WITH_FOXGLOVE:-true}"
FOXGLOVE_BIND="${FOXGLOVE_BIND:-${GO2_IP}}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
WITH_GLASS_AWARE="${WITH_GLASS_AWARE:-true}"
WITH_EDGE_UPLINK="${WITH_EDGE_UPLINK:-false}"
WITH_MOTOR_MONITOR="${WITH_MOTOR_MONITOR:-false}"
WITH_SEMANTIC_OBJNAV="${WITH_SEMANTIC_OBJNAV:-false}"
WITH_SEMANTIC_BEV="${WITH_SEMANTIC_BEV:-false}"
DEMO_PROFILE="${DEMO_PROFILE:-demo_short}"
WITH_LOOP_DETECTION="${WITH_LOOP_DETECTION:-auto}"
AUTOSTART="${AUTOSTART:-true}"
WITH_PCT_PLANNER="${WITH_PCT_PLANNER:-true}"
SESSION_ID="${SESSION_ID:-demo_session}"
EDGE_HOST="${EDGE_HOST:-${EDGE_HOST}}"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-}"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DOCKER="/ws/logs/demo_scan_nav_${TS}.log"

in_container() {
  [[ -f /opt/ros/humble/setup.bash && -f /ws/install/setup.bash ]]
}

container_running() {
  docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${NAME}"
}

container_exists() {
  docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "${NAME}"
}

ensure_container() {
  if in_container; then
    return 0
  fi
  if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: 需要 docker（宿主机执行 ./scripts/motionslam nav start）" >&2
    exit 1
  fi
  if container_running; then
    echo "[motionslam] 容器已在运行"
    return 0
  fi
  if container_exists; then
    echo "[motionslam] 启动已有容器 ${NAME} ..."
    docker start "${NAME}" >/dev/null
  else
    if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
      echo "ERROR: 镜像 ${IMAGE} 不存在。先: docker build -t ${IMAGE} ${ROOT}/docker" >&2
      exit 1
    fi
    echo "[motionslam] 创建并启动容器 ${NAME} ..."
    docker run -d \
      --name "${NAME}" \
      --network host \
      --ipc host \
      --pid host \
      -v "${ROOT}:/ws" \
      -v /dev:/dev \
      --privileged \
      "${IMAGE}" \
      bash -lc "sleep infinity" >/dev/null
  fi
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    container_running && return 0
    sleep 1
  done
  echo "ERROR: 容器 ${NAME} 未能进入 Running" >&2
  docker ps -a --filter "name=${NAME}" >&2
  exit 1
}

ARGS=(
  with_foxglove:="${WITH_FOXGLOVE}"
  foxglove_bind:="${FOXGLOVE_BIND}"
  foxglove_port:="${FOXGLOVE_PORT}"
  with_glass_aware:="${WITH_GLASS_AWARE}"
  with_edge_uplink:="${WITH_EDGE_UPLINK}"
  with_motor_monitor:="${WITH_MOTOR_MONITOR}"
  with_semantic_objnav:="${WITH_SEMANTIC_OBJNAV}"
  with_semantic_bev:="${WITH_SEMANTIC_BEV}"
  demo_profile:="${DEMO_PROFILE}"
  with_loop_detection:="${WITH_LOOP_DETECTION}"
  autostart_lifecycle:="${AUTOSTART}"
  with_pct_planner:="${WITH_PCT_PLANNER}"
  session_id:="${SESSION_ID}"
  edge_host:="${EDGE_HOST}"
)

if [[ -n "${WAYPOINTS_FILE}" ]]; then
  ARGS+=(waypoints_file:="${WAYPOINTS_FILE}")
fi
ARGS+=("$@")

echo "=== Demo SCAN 导航 ==="
echo "  Foxglove ws://${FOXGLOVE_BIND}:${FOXGLOVE_PORT}  pct=${WITH_PCT_PLANNER}  autostart=${AUTOSTART}"
echo "  停栈: ./scripts/motionslam nav stop"
echo "  异常日志: ./scripts/motionslam nav watch"
echo ""

ensure_container
"$SCRIPTS/map/stop_unitree_slam.sh"

if in_container; then
  # shellcheck source=/dev/null
  source "${SCRIPTS}/dev/source_ws_env.sh"
  exec ros2 launch motionslam_bringup demo_scan_stack.launch.py "${ARGS[@]}"
fi

docker exec "${NAME}" bash -lc "
  cd /ws && source /ws/scripts/dev/source_ws_env.sh
  nohup bash -lc 'ros2 launch motionslam_bringup demo_scan_stack.launch.py \
    ${ARGS[*]} \
    2>&1 | tee $LOG_DOCKER /tmp/nav_clean.log' \
    > /dev/null 2>&1 &
  echo \$! > /tmp/demo_scan_stack.pid
"

echo "Demo 栈已后台启动"
echo "  Foxglove: ws://${FOXGLOVE_BIND}:${FOXGLOVE_PORT}  Fixed Frame=world"
echo "  日志: $LOG_DOCKER"
echo "  点目标前（容器内）: ros2 topic pub --once /demo/mission/start std_msgs/msg/String \"{data: 'go'}\""
