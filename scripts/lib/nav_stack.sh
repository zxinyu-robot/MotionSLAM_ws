#!/usr/bin/env bash
# Nav2 / Hybrid 栈公共逻辑（stop 可用；start 在 dev 已禁用，见 feature/hybrid-baseline）
set -uo pipefail

NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

nav_in_container() {
  [[ -f /opt/ros/humble/setup.bash && -f /ws/install/setup.bash ]]
}

nav_ensure_docker() {
  if ! docker ps --format '{{.Names}}' | grep -qx motionslam; then
    echo "ERROR: 容器 motionslam 未运行。请先: docker start motionslam" >&2
    return 1
  fi
}

nav_stack_refused_msg() {
  cat >&2 <<EOF
REFUSED: Nav2 / Hybrid 导航栈在 dev 分支已移除。
  Demo 主线:  ./scripts/motionslam nav start
  Hybrid 对照: git checkout feature/hybrid-baseline
  建图试走:    ./scripts/map/start_mapping_walk.sh
EOF
}

nav_stack_stop() {
  if nav_in_container; then
    bash "${NAV_ROOT}/scripts/nav/stop_nav.sh"
    return $?
  fi
  nav_ensure_docker || return 1
  "${NAV_ROOT}/scripts/map/stop_unitree_slam.sh" 2>/dev/null || true
  docker exec motionslam bash -lc 'bash /ws/scripts/nav/stop_nav.sh'
}

nav_stack_start_bg() {
  nav_stack_refused_msg
  return 2
}

nav_stack_wait_ready() {
  nav_stack_refused_msg
  return 2
}
