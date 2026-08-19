#!/usr/bin/env bash
# F 系列前段验收 — dev 主线 (Demo SCAN + Super-LIO)
# 用法:
#   ./scripts/verify/verify_f_series.sh              # F1 + F2(15s) + F3 Demo + F4 Demo
#   ./scripts/verify/verify_f_series.sh --full       # F2 静止 60s
#   ./scripts/verify/verify_f_series.sh --line       # F2 直线回测
#   ./scripts/verify/verify_f_series.sh --smoke      # E2E 冒烟 (原 e2e_verify.sh)
#   ./scripts/verify/verify_f_series.sh --docker     # 宿主机 docker exec (原 verify_f_series_host.sh)
# 等价: ./scripts/motionslam verify f-series|smoke [--docker]
# 历史 ego/Nav2 F3/F4: feature/hybrid-baseline + scripts/legacy/nav2/
set -eo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

YEAR="$(date +%Y)"
if [[ "$YEAR" -lt 2020 ]]; then
  echo "ERROR: 系统时间异常 ($(date)), 请先: ./scripts/ops/fix_system_time.sh" >&2
  exit 1
fi

FULL=false
LINE_ONLY=false
SMOKE=false
DOCKER=false
LINE_TARGET_M=5.0
LINE_RETURN_TOL=""
ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --full) FULL=true ;;
    --line) LINE_ONLY=true ;;
    --smoke) SMOKE=true ;;
    --docker) DOCKER=true ;;
    --target-m)
      LINE_TARGET_M="${2:?--target-m 需要米数}"
      shift
      ;;
    --return-tol)
      LINE_RETURN_TOL="${2:?--return-tol 需要米数}"
      shift
      ;;
    --f-series) ;; # 兼容 e2e_verify.sh --f-series
    *) ARGS+=("$1") ;;
  esac
  shift
done

if $DOCKER; then
  INNER=()
  $SMOKE && INNER+=(--smoke)
  $FULL && INNER+=(--full)
  $LINE_ONLY && INNER+=(--line)
  [[ "$LINE_TARGET_M" != "5.0" ]] && INNER+=(--target-m "$LINE_TARGET_M")
  [[ -n "$LINE_RETURN_TOL" ]] && INNER+=(--return-tol "$LINE_RETURN_TOL")
  INNER+=("${ARGS[@]}")
  if [[ -f /.dockerenv ]] || { ! command -v docker >/dev/null 2>&1 && [[ -f "$ROOT/install/setup.bash" ]]; }; then
    exec "$SCRIPTS/verify/verify_f_series.sh" "${INNER[@]}"
  elif command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx motionslam; then
    exec docker exec motionslam bash -lc \
      "cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash && \
       ./scripts/verify/verify_f_series.sh $(printf '%q ' "${INNER[@]}")"
  else
    echo "ERROR: --docker 需要 motionslam 容器运行，或已在容器内直接执行本脚本" >&2
    exit 1
  fi
fi

if [[ -z "$LINE_RETURN_TOL" ]]; then
  LINE_RETURN_TOL=$(python3 -c "print(max(0.05, 0.15 * float('$LINE_TARGET_M') / 5.0))")
fi

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
set -e
# shellcheck source=/dev/null
source install/setup.bash 2>/dev/null || {
  echo "ERROR: 请先 colcon build 并 source install/setup.bash" >&2
  exit 1
}
if [[ -f scan_planner_ws/install/setup.bash ]]; then
  # shellcheck source=/dev/null
  source scan_planner_ws/install/setup.bash
fi

if $SMOKE; then
  pass=0
  fail=0
  skip=0
  check() {
    local id="$1" desc="$2"
    shift 2
    if "$@"; then
      echo "[PASS] $id $desc"
      pass=$((pass + 1))
    else
      echo "[FAIL] $id $desc"
      fail=$((fail + 1))
    fi
  }
  skip_item() {
    echo "[SKIP] $1 $2"
    skip=$((skip + 1))
  }
  topic_once_ok() { timeout 3 ros2 topic echo "$1" --once >/dev/null 2>&1; }

  echo "=== MotionSLAM E2E verify (Demo) $(date -Iseconds) ==="
  check F3a "cloud_world 有数据" topic_once_ok /lio/cloud_world
  check F3b "robo/odom 有数据" topic_once_ok /lio/robo/odom
  check F1a "lio/odom 有输出" topic_once_ok /lio/odom
  if topic_once_ok /utlidar/robot_odom; then
    check F1 "LIO/Unitree pitch 差 < 3°" python3 scripts/verify/calibrate_odom_robo.py --once --report-only
  else
    skip_item F1 "无 /utlidar/robot_odom"
  fi
  if topic_once_ok /lio/robo/odom; then
    check F2-smoke "静止 10s 漂移 < 0.05m" \
      python3 scripts/verify/verify_f2_drift.py --static --duration 10 --max-drift 0.05
  else
    skip_item F2-smoke "无 /lio/robo/odom"
  fi
  if ros2 node list 2>/dev/null | grep -q scan_planner_node; then
    check F4-smoke "SCAN bspline" python3 scripts/verify/verify_f4_demo_scan.py --once
  else
    skip_item F4-smoke "无 scan_planner (启动 demo_scan_stack)"
  fi
  check H3 "motion/command 话题存在" bash -c 'ros2 topic list | grep -qx /motion/command'
  skip_item H5-Demo "端到端: python3 scripts/verify.py h5 --distance 2"
  skip_item H5-Nav2 "Nav2 H5: scripts/verify/verify_h5_arrival.py (hybrid 分支)"
  skip_item F2-line "直线: ./scripts/verify/verify_f_series.sh --line"
  echo "=== 汇总: PASS=$pass FAIL=$fail SKIP=$skip ==="
  echo "完整 F 系列: ./scripts/motionslam verify f-series --full"
  [[ $fail -eq 0 ]]
  exit
fi

pass=0
fail=0
skip=0

run_check() {
  local id="$1" desc="$2"
  shift 2
  set +e
  "$@"
  local rc=$?
  set -e
  if [[ $rc -eq 0 ]]; then
    echo "[PASS] $id $desc"
    pass=$((pass + 1))
  elif [[ $rc -eq 2 ]]; then
    echo "[SKIP] $id $desc"
    skip=$((skip + 1))
  else
    echo "[FAIL] $id $desc (exit $rc)"
    fail=$((fail + 1))
  fi
}

echo "=== F 系列验收 (Demo) $(date -Iseconds) full=$FULL line_only=$LINE_ONLY ==="

topic_ok() {
  timeout "${2:-8}" ros2 topic echo "$1" --once >/dev/null 2>&1
}

node_ok() {
  ros2 node list 2>/dev/null | grep -q "$1"
}

print_stack_help() {
  cat <<'EOF'

前置栈未就绪。请另开终端启动:

  source install/setup.bash
  source scan_planner_ws/install/setup.bash   # F4 需要

  # 建图 / 标定最低要求:
  ros2 launch motionslam_bringup mapping.launch.py
  # 或 Demo 导航:
  ./scripts/motionslam nav start

宿主机: ./scripts/motionslam nav start
EOF
}

preflight() {
  echo "--- 前置检查 ---"
  local robo=false scan=false closed=false
  topic_ok /lio/robo/odom 8 && robo=true
  node_ok scan_planner_node && scan=true
  node_ok closed_loop_controller && closed=true

  printf "  /lio/robo/odom:           %s\n" "$($robo && echo OK || echo MISSING)"
  printf "  scan_planner_node:        %s\n" "$($scan && echo OK || echo MISSING)"
  printf "  closed_loop_controller:   %s\n" "$($closed && echo OK || echo MISSING)"

  if ! $robo; then
    print_stack_help
    exit 1
  fi
  if ! $scan; then
    echo ""
    echo "WARN: 无 scan_planner_node → F4 将 SKIP。请 ./scripts/motionslam nav start"
  fi
  echo ""
}

preflight

if $LINE_ONLY; then
  echo "F2-line: 目标 ${LINE_TARGET_M}m, 回起点误差限 ${LINE_RETURN_TOL}m"
  run_check F2-line "直线 ${LINE_TARGET_M}m 回测" \
    python3 scripts/verify/verify_f2_drift.py --line --target-m "$LINE_TARGET_M" --return-tol "$LINE_RETURN_TOL"
  echo "=== 汇总: PASS=$pass FAIL=$fail SKIP=$skip ==="
  [[ $fail -eq 0 ]]
  exit
fi

if topic_ok /utlidar/robot_odom 5; then
  run_check F1 "LIO/Unitree pitch 差 < 3°" python3 scripts/verify/calibrate_odom_robo.py --once --report-only
else
  echo "[SKIP] F1 无 /utlidar/robot_odom"
  skip=$((skip + 1))
fi

STATIC_DUR=15
$FULL && STATIC_DUR=60
run_check F2-static "静止 ${STATIC_DUR}s 漂移 < 0.05m" \
  python3 scripts/verify/verify_f2_drift.py --static --duration "$STATIC_DUR" --max-drift 0.05

if $FULL; then
  echo "[INFO] F2 直线: ./scripts/verify/verify_f_series.sh --line --target-m 3"
  skip=$((skip + 1))
  echo "[SKIP] F2-line 需遥控 (--line)"
else
  echo "[SKIP] F2-line 快速模式跳过 (--full + --line)"
  skip=$((skip + 1))
fi

if topic_ok /lio/cloud_world 5; then
  run_check F3 "cloud_world 障碍带点云" python3 scripts/verify/verify_f3_demo_cloud.py --once
else
  echo "[SKIP] F3 无 /lio/cloud_world"
  skip=$((skip + 1))
fi

if node_ok scan_planner_node; then
  run_check F4 "SCAN goal → bspline" python3 scripts/verify/verify_f4_demo_scan.py --once
else
  echo "[SKIP] F4 无 scan_planner_node"
  skip=$((skip + 1))
fi

echo "=== 汇总: PASS=$pass FAIL=$fail SKIP=$skip ==="
echo "H5 到达 (Demo): python3 scripts/verify.py h5 --distance 2"
echo "H5 到达 (Nav2, hybrid): python3 scripts/verify/verify_h5_arrival.py --distance 2"
[[ $fail -eq 0 ]]
