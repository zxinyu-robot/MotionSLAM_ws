#!/usr/bin/env bash
# NAV-4 WiFi/远程链路中断补采
# 用法:
#   DEV_IP=${DEV_IP} ./scripts/run_nav4_wifi_disconnect_capture.sh
#   # 或手动断网:
#   ./scripts/run_nav4_wifi_disconnect_capture.sh
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

DISTANCE="${DISTANCE:-3}"
DISCONNECT_AT="${DISCONNECT_AT:-8}"
DEV_IP="${DEV_IP:-}"
RELOC_SPOT="${RELOC_SPOT:-map_origin}"
ICP_WAIT="${ICP_WAIT:-12}"
GIT="$(evcap_git_short)"
DATE="$(evcap_date_tag)"
TS="$(evcap_ts)"
BAG_NAME="${DATE}_NAV4_wifi_${GIT}"
BAG_DOCKER="/ws/bags/${BAG_NAME}"
BAG_HOST="${ROOT}/bags/${BAG_NAME}"
LOG_HOST="${ROOT}/logs/nav_${DATE}_nav4_wifi_${TS}.log"
MARKER_HOST="${ROOT}/logs/nav_${DATE}_nav4_disconnect_${TS}.txt"
META_HOST="${ROOT}/export/evidence_${DATE}/META_NAV4_${TS}.txt"
mkdir -p "${ROOT}/bags" "${ROOT}/logs" "${ROOT}/export/evidence_${DATE}"

evcap_ensure_docker
echo "=== NAV-4 断网降级补采 ==="
echo "  t=${DISCONNECT_AT}s 断网; DEV_IP=${DEV_IP:-手动}"
if [[ -z "${DEV_IP}" ]]; then
  echo "  未设 DEV_IP: 脚本会在 t=${DISCONNECT_AT}s 提示手动断开发机 WiFi"
fi

evcap_stop_stacks
evcap_start_nav2 "${RELOC_SPOT}" >/dev/null
evcap_wait_nav2_ready "${ICP_WAIT}"

evcap_start_bag "${BAG_DOCKER}"
sleep 2

IP_ARGS=""
[[ -n "${DEV_IP}" ]] && IP_ARGS="--dev-ip ${DEV_IP}"

set +e
docker exec motionslam bash -lc "
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  python3 /ws/scripts/legacy/nav2/verify_nav4_wifi_disconnect.py \
    --distance ${DISTANCE} \
    --disconnect-at ${DISCONNECT_AT} \
    --marker-file /ws/logs/nav4_marker_${TS}.txt \
    ${IP_ARGS}
" 2>&1 | tee "${LOG_HOST}"
RC=${PIPESTATUS[0]}
set -e

evcap_stop_bag
docker cp "motionslam:/ws/logs/nav4_marker_${TS}.txt" "${MARKER_HOST}" 2>/dev/null || true
evcap_copy_bag_to_host "${BAG_DOCKER}" "${BAG_HOST}" || true

CONCLUSION="$(grep 'conclusion=' "${MARKER_HOST}" 2>/dev/null | tail -1 | cut -d= -f2- || echo unknown)"
RESULT="PARTIAL"
[[ "${RC}" -eq 0 ]] && RESULT="PASS"

{
  echo "date_time=$(date -Iseconds)"
  echo "case_id=NAV-4"
  echo "stack=pure_nav2"
  echo "git_head=${GIT}"
  echo "map_set=$(evcap_map_md5_line)"
  echo "disconnect_at_sec=${DISCONNECT_AT}"
  echo "dev_ip=${DEV_IP:-manual}"
  echo "disconnect_marker=${MARKER_HOST}"
  echo "conclusion=${CONCLUSION}"
  echo "result=${RESULT}"
  echo "bag=${BAG_NAME}"
  echo "log=${LOG_HOST}"
  echo "notes=断网后 estop 已在 verify 结束时探针; 请确认 obstacles_avoid 仍有效"
} > "${META_HOST}"

docker exec motionslam bash -lc 'bash /ws/scripts/nav/stop_nav.sh' 2>/dev/null || true

echo ""
echo "=== NAV-4 完成 exit=${RC} conclusion=${CONCLUSION} ==="
echo "  断网标记: ${MARKER_HOST}"
echo "  bag: ${BAG_HOST}"
exit "${RC}"
