#!/usr/bin/env bash
# Step 1a 验收辅助 — 见 docs/规划/项目规划_Demo1三步.md §1a
# 用法:
#   ./scripts/verify/verify_step1_lean_baseline.sh
#   ./scripts/verify/verify_step1_lean_baseline.sh --record   # 写入 docs/测试验收/step1_baseline_*.txt
#   ./scripts/verify/verify_step1_lean_baseline.sh --docker  # 宿主机 docker exec
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

RECORD=false
DOCKER=false
BSPLINE_WAIT=15
HZ_SAMPLES=5

while [[ $# -gt 0 ]]; do
  case "$1" in
    --record) RECORD=true ;;
    --docker) DOCKER=true ;;
    --bspline-wait)
      BSPLINE_WAIT="${2:?}"
      shift
      ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
done

if $DOCKER; then
  INNER=(--bspline-wait "$BSPLINE_WAIT")
  $RECORD && INNER+=(--record)
  if [[ -f /.dockerenv ]] || { ! command -v docker >/dev/null 2>&1 && [[ -f "$ROOT/install/setup.bash" ]]; }; then
    exec "$SCRIPTS/verify/verify_step1_lean_baseline.sh" "${INNER[@]}"
  elif command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx motionslam; then
    exec docker exec motionslam bash -lc \
      "cd /ws && ./scripts/verify/verify_step1_lean_baseline.sh $(printf '%q ' "${INNER[@]}")"
  else
    echo "ERROR: --docker 需要 motionslam 容器运行，或已在容器内直接执行" >&2
    exit 1
  fi
fi

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
set -e
# shellcheck source=/dev/null
source "$SCRIPTS/dev/source_ws_env.sh" 2>/dev/null || {
  source install/setup.bash 2>/dev/null || {
    echo "ERROR: 请先 colcon build 并 source install/setup.bash" >&2
    exit 1
  }
  [[ -f scan_planner_ws/install/setup.bash ]] && source scan_planner_ws/install/setup.bash
}

OUT=""
if $RECORD; then
  TS="$(date +%Y%m%d_%H%M%S)"
  OUT="$ROOT/docs/测试验收/step1_baseline_${TS}.txt"
  mkdir -p "$(dirname "$OUT")"
  exec > >(tee "$OUT") 2>&1
  echo "记录文件: $OUT"
fi

PASS=0
FAIL=0
WARN=0

check() {
  local id="$1" desc="$2"
  shift 2
  if "$@"; then
    echo "[PASS] $id $desc"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] $id $desc"
    FAIL=$((FAIL + 1))
  fi
}

warn() {
  local id="$1" desc="$2"
  echo "[WARN] $id $desc（需真机/空场人工确认）"
  WARN=$((WARN + 1))
}

echo "=== Step 1 Lean 基线验收 ==="
echo "时间: $(date -Iseconds)"
echo ""

# S1-1：关键节点在线
echo "--- S1-1 栈能启动 ---"
REQUIRED_NODES=(
  demo_scan_bt_orchestrator
  cmd_vel_forwarder
  scan_planner_node
  closed_loop_controller
  super_lio_node
)
for n in "${REQUIRED_NODES[@]}"; do
  if ros2 node list 2>/dev/null | grep -qx "/$n"; then
    check "S1-1" "节点 $n 在线" true
  else
    check "S1-1" "节点 $n 在线" false
  fi
done

# S1-4 部分：单链 /motion/command（closed_loop → forwarder）
echo ""
echo "--- S1-4 /motion/command 单链（订阅数） ---"
MOTION_TOPIC="/motion/command"
CMD_VEL_SUBS="$(ros2 topic info "$MOTION_TOPIC" -v 2>/dev/null | awk '/Subscription count:/ {print $3; exit}')"
PUB_COUNT="$(ros2 topic info "$MOTION_TOPIC" -v 2>/dev/null | awk '/Publisher count:/ {print $3; exit}')"
if [[ -n "${CMD_VEL_SUBS:-}" && -n "${PUB_COUNT:-}" ]]; then
  if [[ "$CMD_VEL_SUBS" -eq 1 && "$PUB_COUNT" -eq 1 ]]; then
    check "S1-4" "${MOTION_TOPIC} pub=1 sub=1" true
  else
    echo "[FAIL] S1-4 ${MOTION_TOPIC} pub=${PUB_COUNT} sub=${CMD_VEL_SUBS}（期望各 1）"
    FAIL=$((FAIL + 1))
  fi
  ros2 topic info "$MOTION_TOPIC" -v 2>/dev/null | sed -n '/Publisher count:/,/Liveliness/p' || true
else
  echo "[FAIL] S1-4 无法读取 ${MOTION_TOPIC} 话题信息"
  FAIL=$((FAIL + 1))
fi

# S1-3：规划输出（bspline 为事件型话题，优先 echo --once，失败则查 launch 日志）
echo ""
echo "--- S1-3 /planning/bspline ---"
S1_3_OK=false
if ros2 topic list 2>/dev/null | grep -qx '/planning/bspline'; then
  echo "等待 ${BSPLINE_WAIT}s 尝试 echo --once（发 goal / replan 成功时才有消息）..."
  if timeout "$((BSPLINE_WAIT + 5))" ros2 topic echo /planning/bspline scan_planner_msgs/msg/Bspline --once >/dev/null 2>&1; then
    S1_3_OK=true
  fi
  if ! $S1_3_OK; then
    LATEST_LOG="$(ls -t /ws/logs/demo_scan_nav_*.log "$ROOT"/logs/demo_scan_nav_*.log 2>/dev/null | head -1 || true)"
    if [[ -n "$LATEST_LOG" && -f "$LATEST_LOG" ]] \
      && grep -qE 'Received trajectory|final_plan_success=1' "$LATEST_LOG" 2>/dev/null; then
      echo "日志 ${LATEST_LOG} 含成功规划记录"
      S1_3_OK=true
    fi
  fi
  if $S1_3_OK; then
    check "S1-3" "/planning/bspline 有输出（echo 或日志）" true
    timeout 8 ros2 topic hz /planning/bspline --window 3 2>&1 | tail -3 || true
  else
    echo "[FAIL] S1-3 未检测到 bspline（确认 autostart 或手动 pub /demo/mission/start）"
    FAIL=$((FAIL + 1))
  fi
else
  echo "[FAIL] S1-3 话题 /planning/bspline 不存在"
  FAIL=$((FAIL + 1))
fi

# S1-5：cloud 订阅 baseline
echo ""
echo "--- S1-5 /lio/cloud_world 订阅 baseline ---"
if ros2 topic list 2>/dev/null | grep -qx '/lio/cloud_world'; then
  ros2 topic info /lio/cloud_world -v 2>/dev/null || true
  SUB_COUNT="$(ros2 topic info /lio/cloud_world -v 2>/dev/null | awk '/Subscription count:/ {print $3; exit}')"
  if [[ -n "${SUB_COUNT:-}" ]]; then
    check "S1-5" "cloud_world 订阅数已记录 (${SUB_COUNT})" true
  else
    echo "[FAIL] S1-5 无法读取 cloud_world 订阅数"
    FAIL=$((FAIL + 1))
  fi
else
  echo "[FAIL] S1-5 话题 /lio/cloud_world 不存在"
  FAIL=$((FAIL + 1))
fi

# S1-2 / S1-6：需真机 soak + tegrastats
echo ""
echo "--- S1-2 / S1-6 人工项 ---"
warn "S1-2" "lean 栈空场连续 ≥10 min 无 OOM（本脚本不自动 soak）"
warn "S1-4" "H5 或 demo waypoints 2m ≥8/10：python3 scripts/verify/verify_scan_planner_h5.py --distance 2"
if command -v tegrastats >/dev/null 2>&1; then
  echo "tegrastats 快照（5s）:"
  timeout 6 tegrastats --interval 1000 2>/dev/null | head -3 || true
  warn "S1-6" "将 tegrastats / 上述输出写入 docs/测试验收/测试记录.md §Step 1"
elif [[ -f /usr/bin/tegrastats ]]; then
  echo "tegrastats（宿主机）:"
  timeout 6 /usr/bin/tegrastats --interval 1000 2>/dev/null | head -3 || true
  warn "S1-6" "将 tegrastats 写入测试记录.md §Step 1"
else
  echo "free -h:"
  free -h || true
  warn "S1-6" "记录 free -h / top 快照到测试记录.md"
fi
echo "S1-2 soak: ./scripts/verify/verify_step1_soak.sh --minutes 10 --record"

echo ""
echo "=== 汇总: PASS=$PASS FAIL=$FAIL WARN=$WARN ==="
if [[ $FAIL -eq 0 ]]; then
  echo "自动化项通过。完成 S1-2/S1-4/S1-6 人工项后在路线图 Step 1 打勾。"
  exit 0
fi
echo "存在 FAIL，请先修复栈再复测。"
exit 1
