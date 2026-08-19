#!/usr/bin/env bash
# Step 2 验收辅助 — MVPI1 定向到达
# 用法:
#   ./scripts/verify/verify_step2_semantic_nav.sh --nav-x 3.0 --nav-y 1.0
#   ./scripts/verify/verify_step2_semantic_nav.sh --reject-test
#   ./scripts/verify/verify_step2_semantic_nav.sh --docker
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

DOCKER=false
REJECT_TEST=false
DIST=2.0
NAV_X=""
NAV_Y=""
SESSION_ID="${SESSION_ID:-demo_live}"
WAIT=90

while [[ $# -gt 0 ]]; do
  case "$1" in
    --docker) DOCKER=true ;;
    --reject-test) REJECT_TEST=true ;;
    --distance) DIST="${2:?}"; shift ;;
    --nav-x) NAV_X="${2:?}"; shift ;;
    --nav-y) NAV_Y="${2:?}"; shift ;;
    --session-id) SESSION_ID="${2:?}"; shift ;;
    --wait) WAIT="${2:?}"; shift ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
done

if $DOCKER; then
  INNER=(--distance "$DIST" --session-id "$SESSION_ID" --wait "$WAIT")
  $REJECT_TEST && INNER+=(--reject-test)
  [[ -n "$NAV_X" ]] && INNER+=(--nav-x "$NAV_X")
  [[ -n "$NAV_Y" ]] && INNER+=(--nav-y "$NAV_Y")
  if [[ -f /.dockerenv ]] || { ! command -v docker >/dev/null 2>&1 && [[ -f "$ROOT/install/setup.bash" ]]; }; then
    exec "$SCRIPTS/verify/verify_step2_semantic_nav.sh" "${INNER[@]}"
  elif docker ps --format '{{.Names}}' 2>/dev/null | grep -qx motionslam; then
    exec docker exec motionslam bash -lc \
      "cd /ws && ./scripts/verify/verify_step2_semantic_nav.sh $(printf '%q ' "${INNER[@]}")"
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

echo "=== Step 2 MVPI1 定向到达验收 ==="
echo "时间: $(date -Iseconds)  session=${SESSION_ID}"

# 前置节点
echo ""
echo "--- 前置：semantic 栈节点 ---"
for n in directive_receiver command_executor demo_scan_bt_orchestrator; do
  if ros2 node list 2>/dev/null | grep -qx "/$n"; then
    check "S2-0" "节点 $n 在线" true
  else
    check "S2-0" "节点 $n 在线" false
  fi
done

# lifecycle → active
echo ""
echo "--- lifecycle configure → activate ---"
ros2 topic pub --once /demo/lifecycle/transition std_msgs/msg/String "{data: 'configure'}" >/dev/null 2>&1 || true
sleep 1
ros2 topic pub --once /demo/lifecycle/transition std_msgs/msg/String "{data: 'activate'}" >/dev/null 2>&1 || true
sleep 2
LC="$(timeout 5 ros2 topic echo /demo/lifecycle/state std_msgs/msg/String --once 2>/dev/null | awk '/data:/ {print $2; exit}')"
if [[ "$LC" == "active" ]]; then
  check "S2-0" "lifecycle=active" true
else
  echo "[WARN] lifecycle=${LC:-unknown}（继续尝试 mock）"
fi

if $REJECT_TEST; then
  echo ""
  echo "--- S2-6 reject：bad session ---"
  OX="$(timeout 5 ros2 topic echo /lio/robo/odom nav_msgs/msg/Odometry --once 2>/dev/null | awk '/x:/{getline; print $2; exit}')"
  OY="$(timeout 5 ros2 topic echo /lio/robo/odom nav_msgs/msg/Odometry --once 2>/dev/null | awk '/y:/{getline; print $2; exit}')"
  python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
    --kind action_group --host 127.0.0.1 \
    --nav-x "${OX:-2.0}" --nav-y "${OY:-0.0}" \
    --map-version 0 --bad-session 2>&1 | tail -3
  sleep 3
  REJ_LOG="$(mktemp)"
  timeout 10 ros2 topic echo /demo/mission/event std_msgs/msg/String 2>/dev/null >"$REJ_LOG" || true
  LATEST_LOG="$(ls -t /ws/logs/demo_scan_nav_*.log 2>/dev/null | head -1 || true)"
  [[ -n "$LATEST_LOG" && -f "$LATEST_LOG" ]] && grep -E "task_rejected|REJECT_SESSION" "$LATEST_LOG" >>"$REJ_LOG" 2>/dev/null || true
  if grep -qE 'task_rejected|REJECT_SESSION' "$REJ_LOG"; then
    check "S2-6" "bad-session → task_rejected" true
  else
    check "S2-6" "bad-session → task_rejected" false
  fi
  rm -f "$REJ_LOG"
  echo "=== 汇总: PASS=$PASS FAIL=$FAIL ==="
  exit $([[ $FAIL -eq 0 ]] && echo 0 || echo 1)
fi

# 计算 NAV 目标（相对当前位姿前进 DIST m）
echo ""
echo "--- 等待 odom ---"
OX="" OY="" YAW=""
for _ in $(seq 1 20); do
  ODOM="$(timeout 3 ros2 topic echo /lio/robo/odom nav_msgs/msg/Odometry --once 2>/dev/null || true)"
  if [[ -n "$ODOM" ]]; then
    OX="$(echo "$ODOM" | awk '/position:/{p=1} p&&/x:/{print $2; exit}')"
    OY="$(echo "$ODOM" | awk '/position:/{p=1} p&&/y:/{print $2; exit}')"
    break
  fi
  sleep 1
done
if [[ -z "$OX" || -z "$OY" ]]; then
  echo "[FAIL] 无 /lio/robo/odom"
  exit 1
fi
echo "  起点 (${OX}, ${OY})"

if [[ -n "$NAV_X" && -n "$NAV_Y" ]]; then
  GX="$NAV_X"
  GY="$NAV_Y"
else
  GX="$(python3 -c "print(float('$OX') + float('$DIST'))")"
  GY="$OY"
fi
echo "  mock NAV → (${GX}, ${GY})"

# 注入 ActionGroup
python3 "$SCRIPTS/offline/mock_semantic_directive.py" \
  --kind action_group --host 127.0.0.1 \
  --session-id "$SESSION_ID" \
  --map-version 0 --context-generation 0 \
  --nav-x "$GX" --nav-y "$GY" --query "Step2定向到达" 2>&1 | tail -5

echo ""
echo "--- 观测事件链 (${WAIT}s) ---"
EVENT_LOG="$(mktemp)"
timeout "$WAIT" ros2 topic echo /demo/mission/event std_msgs/msg/String 2>/dev/null >"$EVENT_LOG" || true
# echo 可能因 QoS/负载漏消息，回退 launch 日志
LATEST_LOG="$(ls -t /ws/logs/demo_scan_nav_*.log "$ROOT"/logs/demo_scan_nav_*.log 2>/dev/null | head -1 || true)"
if [[ -n "$LATEST_LOG" && -f "$LATEST_LOG" ]]; then
  cat "$LATEST_LOG" >>"$EVENT_LOG"
fi

grep -o '"event": "[^"]*"' "$EVENT_LOG" | sort -u | sed 's/"event": "/  /' | sed 's/"$//' || true

if grep -qE 'task_accepted|EXEC_FEEDBACK event=task_accepted' "$EVENT_LOG"; then
  check "S2-1" "task_accepted" true
else
  check "S2-1" "task_accepted" false
fi
if grep -qE 'semantic_goal_armed|EXEC_FEEDBACK event=semantic_goal_armed' "$EVENT_LOG"; then
  check "S2-2" "semantic_goal_armed" true
else
  check "S2-2" "semantic_goal_armed" false
fi
if grep -qE 'subgoal_dispatched|scan_planner_ready' "$EVENT_LOG"; then
  check "S2-3" "SCAN dispatch" true
else
  check "S2-3" "SCAN dispatch" false
fi
if grep -qE 'subgoal_reached|object_found|SubgoalReached' "$EVENT_LOG"; then
  check "S2-4" "到达事件" true
else
  check "S2-4" "到达事件" false
fi

rm -f "$EVENT_LOG"
echo ""
echo "=== 汇总: PASS=$PASS FAIL=$FAIL ==="
if [[ $FAIL -eq 0 ]]; then
  echo "Step 2 事件链通过。S2-5 成功率与 S2-7/S2-8 需另行批量/edge 验证。"
  exit 0
fi
exit 1
