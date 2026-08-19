#!/usr/bin/env bash
# ATE/RPE 补采 — 三场景之一
# 用法:
#   ./scripts/run_ate_rpe_capture.sh straight_3m    # 空场 3m 往返
#   ./scripts/run_ate_rpe_capture.sh n6_3m        # 同 N6 单段 3m
#   ./scripts/run_ate_rpe_capture.sh static_60s     # 静止 60s 漂移
#   ATE_BAG_LITE=0 ./scripts/run_ate_rpe_capture.sh straight_3m  # 含点云
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
source "${ROOT}/scripts/lib/evidence_capture.sh"
# shellcheck source=scripts/lib/nav_stack.sh
source "${ROOT}/scripts/lib/nav_stack.sh"
if [[ "${ALLOW_LEGACY_NAV:-}" != "YES" ]]; then
  nav_stack_refused_msg
  exit 2
fi

SCENARIO="${1:-}"
if [[ -z "${SCENARIO}" ]]; then
  echo "用法: $0 {straight_3m|n6_3m|static_60s}" >&2
  exit 1
fi

RELOC_SPOT="${RELOC_SPOT:-map_origin}"
ICP_WAIT="${ICP_WAIT:-12}"
GIT="$(evcap_git_short)"
DATE="$(evcap_date_tag)"
TS="$(evcap_ts)"
BAG_NAME="${DATE}_ATE_${SCENARIO}_${GIT}"
BAG_DOCKER="/ws/bags/${BAG_NAME}"
BAG_HOST="${ROOT}/bags/${BAG_NAME}"
LOG_HOST="${ROOT}/logs/ate_${DATE}_${SCENARIO}_${TS}.log"
META_HOST="${ROOT}/export/evidence_${DATE}/META_ATE_${SCENARIO}_${TS}.txt"
mkdir -p "${ROOT}/bags" "${ROOT}/logs" "${ROOT}/export/evidence_${DATE}"

case "${SCENARIO}" in
  straight_3m) VERIFY_CMD="python3 /ws/scripts/legacy/nav2/verify_ate_straight_3m.py" ;;
  n6_3m)       VERIFY_CMD="python3 /ws/scripts/verify/verify_h5_arrival.py --distance 3" ;;
  static_60s)  VERIFY_CMD="python3 /ws/scripts/verify/verify_ate_static_drift.py --duration 60" ;;
  *)
    echo "未知场景: ${SCENARIO}" >&2
    exit 1
    ;;
esac

evcap_ensure_docker
echo "=== ATE/RPE 补采: ${SCENARIO} ==="
echo "  bag: ${BAG_HOST}"

if [[ "${SCENARIO}" == "static_60s" ]]; then
  echo "  请将狗置于空场静止位置"
  read -r -p "  已就位? [y/N] " confirm
  [[ "${confirm}" =~ ^[Yy]$ ]] || exit 0
fi

evcap_stop_stacks
evcap_start_nav2 "${RELOC_SPOT}" >/dev/null
evcap_wait_nav2_ready "${ICP_WAIT}"

evcap_start_bag "${BAG_DOCKER}"
sleep 2

set +e
docker exec motionslam bash -lc "
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  ${VERIFY_CMD}
" 2>&1 | tee "${LOG_HOST}"
RC=${PIPESTATUS[0]}
set -e

evcap_stop_bag
evcap_copy_bag_to_host "${BAG_DOCKER}" "${BAG_HOST}" || true

RESULT="FAIL"
[[ "${RC}" -eq 0 ]] && RESULT="PASS"

{
  echo "date_time=$(date -Iseconds)"
  echo "case_id=ATE-${SCENARIO}"
  echo "stack=pure_nav2"
  echo "git_head=${GIT}"
  echo "map_set=$(evcap_map_md5_line)"
  echo "scenario=${SCENARIO}"
  echo "verify_cmd=${VERIFY_CMD}"
  echo "result=${RESULT}"
  echo "ground_truth=无外部真值，仅相对闭环/回测"
  echo "bag=${BAG_NAME}"
  echo "log=${LOG_HOST}"
  echo "evo_topics=/lio/robo/odom /tf /tf_static /cmd_vel"
  echo "notes=离线 evo: ros2 bag play + evo_traj odom /tf"
} > "${META_HOST}"

docker exec motionslam bash -lc 'bash /ws/scripts/nav/stop_nav.sh' 2>/dev/null || true

echo ""
echo "=== ATE ${SCENARIO} 完成 exit=${RC} ==="
echo "  bag:  ${BAG_HOST}"
echo "  META: ${META_HOST}"
exit "${RC}"
