#!/usr/bin/env bash
# Nav2 证据补采 — 公共函数（N6 / NAV-2 / NAV-4 / ATE）
# NOTE: dev 主线已移除 Nav2；evcap_start_nav2 会 REFUSED，请用 Demo 或 feature/hybrid-baseline
set -uo pipefail

EVCAP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/lib/evidence_bag_topics.sh
source "${EVCAP_ROOT}/scripts/lib/evidence_bag_topics.sh"

evcap_git_short() {
  git -C "${EVCAP_ROOT}" rev-parse --short HEAD 2>/dev/null || echo unknown
}

evcap_date_tag() {
  date +%Y%m%d
}

evcap_ts() {
  date +%Y%m%d_%H%M%S
}

evcap_ensure_docker() {
  if ! docker ps --format '{{.Names}}' | grep -qx motionslam; then
    echo "ERROR: 容器 motionslam 未运行。请先: cd ${EVCAP_ROOT} && docker start motionslam" >&2
    return 1
  fi
}

evcap_stop_stacks() {
  # shellcheck source=scripts/lib/nav_stack.sh
  source "${EVCAP_ROOT}/scripts/lib/nav_stack.sh"
  nav_stack_stop
  sleep 2
}

evcap_start_nav2() {
  local reloc_spot="${1:-map_origin}"
  # shellcheck source=scripts/lib/nav_stack.sh
  source "${EVCAP_ROOT}/scripts/lib/nav_stack.sh"
  nav_stack_start_bg "${reloc_spot}" true obstacles_avoid ""
}

evcap_wait_nav2_ready() {
  local icp_wait="${1:-15}"
  # shellcheck source=scripts/lib/nav_stack.sh
  source "${EVCAP_ROOT}/scripts/lib/nav_stack.sh"
  nav_stack_wait_ready "${icp_wait}"
}

evcap_bag_topics() {
  if [[ "${ATE_BAG_LITE:-1}" == "1" ]]; then
    printf '%s\n' "${EVIDENCE_BAG_TOPICS_LITE[@]}"
  else
    printf '%s\n' "${EVIDENCE_BAG_TOPICS_FULL[@]}"
  fi
}

evcap_start_bag() {
  local bag_dir="$1"
  local topics
  topics="$(evcap_bag_topics | tr '\n' ' ')"
  docker exec motionslam bash -lc "
    source /opt/ros/humble/setup.bash
    rm -rf '${bag_dir}'
    nohup ros2 bag record -o '${bag_dir}' ${topics} \
      > /tmp/evidence_bag_record.log 2>&1 &
    echo \$! > /tmp/evidence_bag.pid
  "
  echo "[bag] 录制: ${bag_dir}"
}

evcap_stop_bag() {
  docker exec motionslam bash -lc 'pkill -f "ros2 bag record" 2>/dev/null || true'
  sleep 1
}

evcap_write_meta_snippet() {
  local meta_file="$1"
  local case_id="$2"
  local result="$3"
  local notes="$4"
  local start_xy="${5:-}"
  local goal_xy="${6:-}"
  local gt="${7:-无外部真值，仅相对闭环/回测}"
  {
    echo "case_id=${case_id}"
    echo "date_time=$(date -Iseconds)"
    echo "stack=pure_nav2"
    echo "git_head=$(evcap_git_short)"
    echo "start_xy=${start_xy}"
    echo "goal_xy=${goal_xy}"
    echo "ground_truth=${gt}"
    echo "result=${result}"
    echo "notes=${notes}"
    echo "map_md5=$(md5sum "${EVCAP_ROOT}/maps/map_nav.pcd" 2>/dev/null | awk '{print $1}')"
  } >> "${meta_file}"
}

evcap_copy_bag_to_host() {
  local bag_docker="$1"
  local bag_host="$2"
  mkdir -p "$(dirname "${bag_host}")"
  docker cp "motionslam:${bag_docker}" "${bag_host}" 2>/dev/null || {
    echo "WARN: docker cp 失败, bag 仍在容器 ${bag_docker}" >&2
    return 1
  }
}

evcap_map_md5_line() {
  md5sum "${EVCAP_ROOT}/maps/map_reloc.pcd" "${EVCAP_ROOT}/maps/map_nav.pcd" \
    "${EVCAP_ROOT}/maps/map_viz.pcd" 2>/dev/null | awk '{print $1}' | paste -sd'/' -
}
