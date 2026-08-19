#!/usr/bin/env bash
# Step 3 闭环 — 边侧 mock planner → SEARCH → re-detect NAV → object_found
# 用法:
#   终端1（边侧/宿主机）: python3 scripts/offline/edge_mvpi1_planner.py --start-search
#   终端2（GO2 容器）:     ./scripts/motionslam verify step3-loop --docker
#
#   或一键（planner 在宿主机监听，GO2 edge_host 指向宿主机）:
#   ./scripts/motionslam verify step3-loop --docker --with-planner
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

DOCKER=false
WITH_PLANNER=false
SESSION_ID="${SESSION_ID:-demo_live}"
WAIT=300
WARMUP=40
QUERY="找红色灭火器"
EDGE_HOST="${EDGE_HOST:-127.0.0.1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --docker) DOCKER=true ;;
    --with-planner) WITH_PLANNER=true ;;
    --session-id) SESSION_ID="${2:?}"; shift ;;
    --wait) WAIT="${2:?}"; shift ;;
    --warmup) WARMUP="${2:?}"; shift ;;
    --query) QUERY="${2:?}"; shift ;;
    --edge-host) EDGE_HOST="${2:?}"; shift ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
done

if $DOCKER; then
  INNER=(--session-id "$SESSION_ID" --wait "$WAIT" --warmup "$WARMUP" --query "$QUERY" --edge-host "$EDGE_HOST")
  $WITH_PLANNER && INNER+=(--with-planner)
  if [[ -f /.dockerenv ]] || { ! command -v docker >/dev/null 2>&1 && [[ -f "$ROOT/install/setup.bash" ]]; }; then
    exec "$SCRIPTS/verify/verify_step3_search_loop.sh" "${INNER[@]}"
  elif docker ps --format '{{.Names}}' 2>/dev/null | grep -qx motionslam; then
    exec docker exec motionslam bash -lc \
      "cd /ws && ./scripts/verify/verify_step3_search_loop.sh $(printf '%q ' "${INNER[@]}")"
  else
    echo "ERROR: --docker 需要 motionslam 容器" >&2
    exit 1
  fi
fi

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
set -e
# shellcheck source=/dev/null
source "$SCRIPTS/dev/source_ws_env.sh" 2>/dev/null || source install/setup.bash

TS="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/docs/测试验收/step3_loop_${TS}.txt"
mkdir -p "$(dirname "$OUT")"
exec > >(tee "$OUT") 2>&1

PLANNER_PID=""
cleanup() {
  [[ -n "$PLANNER_PID" ]] && kill "$PLANNER_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "=== Step 3 SEARCH→NAV 闭环验收 ==="
echo "时间: $(date -Iseconds)  query=${QUERY}"
echo "记录: $OUT"
echo "等待 LIO ${WARMUP}s..."
sleep "$WARMUP"

ensure_lc() {
  local lc
  lc="$(timeout 5 ros2 topic echo /demo/lifecycle/state std_msgs/msg/String --once 2>/dev/null | awk '/data:/ {print $2; exit}')"
  if [[ "$lc" != "active" ]]; then
    ros2 topic pub --once /demo/lifecycle/transition std_msgs/msg/String "{data: 'configure'}" >/dev/null 2>&1 || true
    sleep 0.5
    ros2 topic pub --once /demo/lifecycle/transition std_msgs/msg/String "{data: 'activate'}" >/dev/null 2>&1 || true
    sleep 2
  fi
}

ensure_lc

if $WITH_PLANNER; then
  echo "--- 启动 edge_mvpi1_planner（后台）---"
  python3 "$SCRIPTS/offline/edge_mvpi1_planner.py" \
    --go2-host 127.0.0.1 --go2-port 9879 \
    --redeploy-uplinks 2 --redeploy-reached 1 \
    --query "$QUERY" --start-search &
  PLANNER_PID=$!
  sleep 1
else
  echo "--- 请确保 edge_mvpi1_planner 已在 ${EDGE_HOST}:9877/9880 运行并已 --start-search ---"
  python3 "$SCRIPTS/offline/edge_mvpi1_planner.py" \
    --go2-host 127.0.0.1 --go2-port 9879 \
    --query "$QUERY" --start-search || true
fi

EVENT_LOG="$(mktemp)"
timeout "$WAIT" ros2 topic echo /demo/mission/event std_msgs/msg/String 2>/dev/null >"$EVENT_LOG" || true
LATEST_LOG="$(ls -t /ws/logs/demo_scan_nav_*.log "$ROOT"/logs/demo_scan_nav_*.log 2>/dev/null | head -1 || true)"
[[ -n "$LATEST_LOG" && -f "$LATEST_LOG" ]] && cat "$LATEST_LOG" >>"$EVENT_LOG" || true

pass() { echo "[PASS] $1 $2"; }
fail() { echo "[FAIL] $1 $2"; }

S3_SEARCH=false
S3_HANDOFF=false
S3_NAV=false
S3_FOUND=false

grep -qE 'search_explore|search_mission_armed|frontier_selected' "$EVENT_LOG" && S3_SEARCH=true
grep -q search_handoff "$EVENT_LOG" && S3_HANDOFF=true
grep -qE 'semantic_goal_armed|search_navigate' "$EVENT_LOG" && S3_NAV=true
grep -q object_found "$EVENT_LOG" && S3_FOUND=true

$S3_SEARCH && pass "S3-loop-1" "SEARCH 探索事件" || fail "S3-loop-1" "SEARCH 探索事件"
$S3_HANDOFF && pass "S3-loop-2" "search_handoff（切 NAV）" || fail "S3-loop-2" "search_handoff（切 NAV）"
$S3_NAV && pass "S3-loop-3" "semantic_goal_armed / search_navigate" || fail "S3-loop-3" "NAV 切换"
$S3_FOUND && pass "S3-loop-4" "object_found" || fail "S3-loop-4" "object_found"

grep -o '"event": "[^"]*"' "$EVENT_LOG" | sort -u | sed 's/"event": "/  /' | sed 's/"$//' || true
rm -f "$EVENT_LOG"

echo ""
if $S3_SEARCH && $S3_NAV && $S3_FOUND; then
  echo "Step 3 闭环通过（MVPI1-B1 子集）。完整 S3-2/S3-4 需真机多 trial。"
  exit 0
fi
exit 1
