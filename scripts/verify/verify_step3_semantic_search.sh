#!/usr/bin/env bash
# Step 3 验收 — MVPI1 主动搜索（端侧 + mock 下行）
# 用法:
#   ./scripts/verify/verify_step3_semantic_search.sh --docker
#   ./scripts/verify/verify_step3_semantic_search.sh --frontier-bias-test --docker
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

DOCKER=false
FRONTIER_BIAS=false
SESSION_ID="${SESSION_ID:-demo_live}"
WAIT=120

while [[ $# -gt 0 ]]; do
  case "$1" in
    --docker) DOCKER=true ;;
    --frontier-bias-test) FRONTIER_BIAS=true ;;
    --session-id) SESSION_ID="${2:?}"; shift ;;
    --wait) WAIT="${2:?}"; shift ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
done

if $DOCKER; then
  INNER=(--session-id "$SESSION_ID" --wait "$WAIT")
  $FRONTIER_BIAS && INNER+=(--frontier-bias-test)
  if [[ -f /.dockerenv ]] || { ! command -v docker >/dev/null 2>&1 && [[ -f "$ROOT/install/setup.bash" ]]; }; then
    exec "$SCRIPTS/verify/verify_step3_semantic_search.sh" "${INNER[@]}"
  elif docker ps --format '{{.Names}}' 2>/dev/null | grep -qx motionslam; then
    exec docker exec motionslam bash -lc \
      "cd /ws && ./scripts/verify/verify_step3_semantic_search.sh $(printf '%q ' "${INNER[@]}")"
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

PASS=0
FAIL=0

check() {
  local id="$1" desc="$2" ok="$3"
  if [[ "$ok" == true ]]; then
    echo "[PASS] $id $desc"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] $id $desc"
    FAIL=$((FAIL + 1))
  fi
}

collect_events() {
  local log="$1"
  timeout "$WAIT" ros2 topic echo /demo/mission/event std_msgs/msg/String 2>/dev/null >"$log" || true
  LATEST_LOG="$(ls -t /ws/logs/demo_scan_nav_*.log "$ROOT"/logs/demo_scan_nav_*.log 2>/dev/null | head -1 || true)"
  [[ -n "$LATEST_LOG" && -f "$LATEST_LOG" ]] && cat "$LATEST_LOG" >>"$log" || true
}

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

echo "=== Step 3 MVPI1 主动搜索验收 ==="
echo "时间: $(date -Iseconds)  session=${SESSION_ID}"

echo ""
echo "--- 前置：semantic 栈节点 ---"
for n in directive_receiver command_executor demo_scan_bt_orchestrator subgraph_publisher; do
  if ros2 node list 2>/dev/null | grep -qx "/$n"; then
    check "S3-0" "节点 $n 在线" true
  else
    check "S3-0" "节点 $n 在线" false
  fi
done

ensure_lc

if $FRONTIER_BIAS; then
  echo ""
  echo "--- S3-1：directive 改变 frontier 选择 ---"
  LOG_A="$(mktemp)"
  LOG_B="$(mktemp)"
  python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
    --kind directive --host 127.0.0.1 --session-id "$SESSION_ID" \
    --map-version 0 --mode explore --frontiers "ray_00" --query "biasA" >/dev/null
  python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
    --kind action_group --host 127.0.0.1 --session-id "$SESSION_ID" \
    --map-version 0 --mode search --query "biasA" >/dev/null
  collect_events "$LOG_A"
  sleep 3
  ros2 topic pub --once /demo/mission/cancel std_msgs/msg/String "{data: 'stop'}" >/dev/null 2>&1 || true
  sleep 3
  ensure_lc
  python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
    --kind directive --host 127.0.0.1 --session-id "$SESSION_ID" \
    --map-version 0 --mode explore --frontiers "ray_04" --query "biasB" >/dev/null
  python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
    --kind action_group --host 127.0.0.1 --session-id "$SESSION_ID" \
    --map-version 0 --mode search --query "biasB" >/dev/null
  collect_events "$LOG_B"
  FA="$(grep -o '"frontier_id": "[^"]*"' "$LOG_A" | head -1 || true)"
  FB="$(grep -o '"frontier_id": "[^"]*"' "$LOG_B" | head -1 || true)"
  echo "  runA frontier: ${FA:-none}"
  echo "  runB frontier: ${FB:-none}"
  if [[ -n "$FA" && -n "$FB" && "$FA" != "$FB" ]]; then
    check "S3-1" "有/无 directive 选不同 frontier" true
  elif grep -q frontier_selected "$LOG_A" && grep -q frontier_selected "$LOG_B"; then
    check "S3-1" "frontier_selected 事件可见" true
  else
    check "S3-1" "directive 改变 frontier 选择" false
  fi
  rm -f "$LOG_A" "$LOG_B"
else
  echo ""
  echo "--- SEARCH mock 事件链 ---"
  EVENT_LOG="$(mktemp)"
  python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
    --kind directive --host 127.0.0.1 --session-id "$SESSION_ID" \
    --map-version 0 --mode explore --frontiers "ray_02" --query "Step3搜索" >/dev/null
  python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
    --kind action_group --host 127.0.0.1 --session-id "$SESSION_ID" \
    --map-version 0 --mode search --query "Step3搜索" >/dev/null
  collect_events "$EVENT_LOG"

  grep -o '"event": "[^"]*"' "$EVENT_LOG" | sort -u | sed 's/"event": "/  /' | sed 's/"$//' || true

  grep -qE 'task_accepted|search_explore|search_mission_armed' "$EVENT_LOG" \
    && check "S3-2a" "SEARCH task_accepted / search_explore" true \
    || check "S3-2a" "SEARCH task_accepted / search_explore" false
  grep -q frontier_selected "$EVENT_LOG" \
    && check "S3-2b" "frontier_selected" true \
    || check "S3-2b" "frontier_selected" false
  grep -q subgoal_dispatched "$EVENT_LOG" \
    && check "S3-2c" "subgoal_dispatched（探索 dispatch）" true \
    || check "S3-2c" "subgoal_dispatched（探索 dispatch）" false
  rm -f "$EVENT_LOG"
fi

echo ""
echo "=== 汇总: PASS=$PASS FAIL=$FAIL ==="
exit $([[ $FAIL -eq 0 ]] && echo 0 || echo 1)
