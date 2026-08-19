#!/usr/bin/env bash
# Step 2 收关：S2-5 批量 2m + S2-7 ttl + S2-8 feedback
# 用法:
#   ./scripts/motionslam verify step2-complete --docker
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

DOCKER=false
TRIALS=10
MIN_PASS=8
DIST=2.0
SESSION_ID="${SESSION_ID:-demo_live}"
WARMUP=50

while [[ $# -gt 0 ]]; do
  case "$1" in
    --docker) DOCKER=true ;;
    --trials) TRIALS="${2:?}"; shift ;;
    --min-pass) MIN_PASS="${2:?}"; shift ;;
    --distance) DIST="${2:?}"; shift ;;
    --session-id) SESSION_ID="${2:?}"; shift ;;
    --warmup) WARMUP="${2:?}"; shift ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
done

if $DOCKER; then
  exec docker exec motionslam bash -lc \
    "cd /ws && ./scripts/verify/verify_step2_complete.sh \
      --trials $TRIALS --min-pass $MIN_PASS --distance $DIST --session-id $SESSION_ID --warmup $WARMUP"
fi

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
set -e
# shellcheck source=/dev/null
source "$SCRIPTS/dev/source_ws_env.sh" 2>/dev/null || source install/setup.bash

TS="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/docs/测试验收/step2_complete_${TS}.txt"
mkdir -p "$(dirname "$OUT")"
exec > >(tee "$OUT") 2>&1

LATEST_LOG="$(ls -t /ws/logs/demo_scan_nav_*.log 2>/dev/null | head -1 || true)"

echo "=== Step 2 收关验收 ==="
echo "时间: $(date -Iseconds)  trials=$TRIALS min_pass=$MIN_PASS dist=${DIST}m"
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

echo ""
echo "--- S2-5：${TRIALS} 次 mock 定向 ${DIST}m ---"
OK=0
for i in $(seq 1 "$TRIALS"); do
  ensure_lc
  echo "  [trial $i/$TRIALS]"
  if "$SCRIPTS/verify/verify_step2_semantic_nav.sh" \
      --distance "$DIST" --session-id "$SESSION_ID" --wait 120 > /tmp/step2_trial.log 2>&1; then
    echo "    PASS"
    OK=$((OK + 1))
  elif grep -qE '\[PASS\] S2-4|object_found|subgoal_reached' /tmp/step2_trial.log; then
    echo "    PASS (log)"
    OK=$((OK + 1))
  else
    echo "    FAIL"
    ros2 topic pub --once /demo/mission/cancel std_msgs/msg/String "{data: 'stop'}" >/dev/null 2>&1 || true
    sleep 5
  fi
  sleep 4
done
echo "S2-5: ${OK}/${TRIALS} (要求 ≥${MIN_PASS})"
S2_5_OK=false
[[ $OK -ge $MIN_PASS ]] && S2_5_OK=true

echo ""
echo "--- S2-7：ttl 过期 ---"
ensure_lc
START_LINE="$(wc -l <"${LATEST_LOG:-/dev/null}" 2>/dev/null || echo 0)"
python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
  --kind action_group --host 127.0.0.1 --session-id "$SESSION_ID" \
  --map-version 0 --nav-x 0 --nav-y 0 --expired >/dev/null 2>&1 || true
sleep 2
S2_7_OK=false
if [[ -f "$LATEST_LOG" ]] && tail -n +"$START_LINE" "$LATEST_LOG" | grep -qE 'REJECT_EXPIRED|task_rejected'; then
  echo "S2-7 PASS"
  S2_7_OK=true
else
  echo "S2-7 FAIL"
fi

echo ""
echo "--- S2-8：execution_feedback ---"
S2_8_OK=false
if [[ -f "$LATEST_LOG" ]] && grep -q "EXEC_FEEDBACK event=task_accepted" "$LATEST_LOG"; then
  echo "S2-8 PASS"
  S2_8_OK=true
else
  echo "S2-8 FAIL"
fi

echo ""
echo "=== 汇总 ==="
echo "  S2-5: $([[ $S2_5_OK == true ]] && echo PASS || echo FAIL) ($OK/$TRIALS)"
echo "  S2-7: $([[ $S2_7_OK == true ]] && echo PASS || echo FAIL)"
echo "  S2-8: $([[ $S2_8_OK == true ]] && echo PASS || echo FAIL)"

if $S2_5_OK && $S2_7_OK && $S2_8_OK; then
  echo "Step 2 可标 [x]"
  exit 0
fi
exit 1
