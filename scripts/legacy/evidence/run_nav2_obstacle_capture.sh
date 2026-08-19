#!/usr/bin/env bash
# NAV-2 静态障碍补采: 纸箱绕行/停住 + bag + 照片提示
# 用法:
#   # 先在路径 1–2 m 处放置纸箱 (高 0.2–1.5 m)
#   ./scripts/run_nav2_obstacle_capture.sh
#   DISTANCE=3 ./scripts/run_nav2_obstacle_capture.sh
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
RELOC_SPOT="${RELOC_SPOT:-map_origin}"
ICP_WAIT="${ICP_WAIT:-12}"
GIT="$(evcap_git_short)"
DATE="$(evcap_date_tag)"
TS="$(evcap_ts)"
BAG_NAME="${DATE}_NAV2_obstacle_${GIT}"
BAG_DOCKER="/ws/bags/${BAG_NAME}"
BAG_HOST="${ROOT}/bags/${BAG_NAME}"
LOG_HOST="${ROOT}/logs/nav_${DATE}_nav2_obstacle_${TS}.log"
META_HOST="${ROOT}/export/evidence_${DATE}/META_NAV2_${TS}.txt"
PHOTO_HINT="${ROOT}/export/evidence_${DATE}/NAV2_photo_${TS}.txt"
mkdir -p "${ROOT}/bags" "${ROOT}/logs" "${ROOT}/export/evidence_${DATE}"

evcap_ensure_docker
echo "=== NAV-2 静态障碍补采 ==="
echo "  请在狗头正前方 ${DISTANCE} m 路径的 1–2 m 处放置纸箱"
if [[ "${SKIP_CONFIRM:-0}" != "1" ]]; then
  read -r -p "  纸箱已就位? [y/N] " confirm
  [[ "${confirm}" =~ ^[Yy]$ ]] || { echo "已取消"; exit 0; }
fi

evcap_stop_stacks
NAV_LOG="$(evcap_start_nav2 "${RELOC_SPOT}")"
evcap_wait_nav2_ready "${ICP_WAIT}"

evcap_start_bag "${BAG_DOCKER}"
sleep 2

set +e
docker exec motionslam bash -lc "
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  python3 /ws/scripts/legacy/nav2/verify_nav2_static_obstacle.py --distance ${DISTANCE}
" 2>&1 | tee "${LOG_HOST}"
RC=${PIPESTATUS[0]}
set -e

evcap_stop_bag
evcap_copy_bag_to_host "${BAG_DOCKER}" "${BAG_HOST}" || true

OUTCOME="$(grep -oE 'PASS \[[^]]+\]|FAIL \[[^]]+\]' "${LOG_HOST}" | head -1 || echo "UNKNOWN")"
RESULT="FAIL"
[[ "${RC}" -eq 0 ]] && RESULT="PASS"

{
  echo "date_time=$(date -Iseconds)"
  echo "case_id=NAV-2"
  echo "stack=pure_nav2"
  echo "git_head=${GIT}"
  echo "map_set=$(evcap_map_md5_line)"
  echo "launch_cmd=REFUSED (dev); use feature/hybrid-baseline or ./scripts/motionslam nav start"
  echo "verify_cmd=python3 scripts/legacy/nav2/verify_nav2_static_obstacle.py --distance ${DISTANCE}"
  echo "result=${RESULT}"
  echo "outcome=${OUTCOME}"
  echo "bag=${BAG_NAME}"
  echo "log=${LOG_HOST}"
  echo "notes=途中纸箱; 可选照片/短视频路径填下方"
  echo "photo_path="
  echo "video_path="
} > "${META_HOST}"

cat > "${PHOTO_HINT}" <<EOF
NAV-2 补采 ${TS}
请将现场照片或短视频路径写入:
  ${META_HOST}
建议拍摄: 纸箱位置、狗绕行/停住瞬间、Foxglove 截图
EOF

docker exec motionslam bash -lc 'bash /ws/scripts/nav/stop_nav.sh' 2>/dev/null || true

echo ""
echo "=== NAV-2 完成 exit=${RC} ${OUTCOME} ==="
echo "  bag:  ${BAG_HOST}"
echo "  log:  ${LOG_HOST}"
echo "  META: ${META_HOST}"
exit "${RC}"
