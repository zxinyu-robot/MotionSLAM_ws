#!/usr/bin/env bash
# Step 1 S1-2：lean 栈 soak 监控（默认 10 min）
# 用法:
#   ./scripts/verify/verify_step1_soak.sh
#   ./scripts/verify/verify_step1_soak.sh --minutes 10 --record
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"

MINUTES=10
INTERVAL=30
RECORD=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --minutes)
      MINUTES="${2:?}"
      shift
      ;;
    --interval)
      INTERVAL="${2:?}"
      shift
      ;;
    --record) RECORD=true ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
done

OUT=""
if $RECORD; then
  TS="$(date +%Y%m%d_%H%M%S)"
  OUT="$ROOT/docs/测试验收/step1_soak_${TS}.txt"
  mkdir -p "$(dirname "$OUT")"
  exec > >(tee "$OUT") 2>&1
  echo "记录文件: $OUT"
fi

END=$((SECONDS + MINUTES * 60))
SAMPLES=0
FAIL=0

echo "=== Step 1 Soak S1-2 ==="
echo "时长: ${MINUTES} min  采样间隔: ${INTERVAL}s"
echo "开始: $(date -Iseconds)"

while [[ $SECONDS -lt $END ]]; do
  SAMPLES=$((SAMPLES + 1))
  echo ""
  echo "--- sample ${SAMPLES} @ $(date -Iseconds) ---"
  if command -v tegrastats >/dev/null 2>&1; then
    timeout 3 tegrastats --interval 1000 2>/dev/null | head -1 || true
  fi
  free -h | awk '/^Mem:/ {print "Mem:", $2, "used", $3, "avail", $7}'
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx motionslam; then
    docker exec motionslam bash -lc '
      source /ws/scripts/dev/source_ws_env.sh 2>/dev/null || exit 0
      for n in demo_scan_bt_orchestrator cmd_vel_forwarder scan_planner_node closed_loop_controller super_lio_node; do
        ros2 node list 2>/dev/null | grep -qx "/$n" || { echo "MISSING_NODE $n"; exit 2; }
      done
      echo "nodes: OK"
    ' || {
      echo "[FAIL] 关键节点离线"
      FAIL=$((FAIL + 1))
    }
  else
    echo "[WARN] motionslam 容器未运行"
  fi
  REMAIN=$((END - SECONDS))
  [[ $REMAIN -le 0 ]] && break
  sleep "$INTERVAL"
done

echo ""
echo "=== Soak 结束: samples=$SAMPLES node_fail=$FAIL ==="
if [[ $FAIL -eq 0 ]]; then
  echo "S1-2 自动化监控通过（仍需确认无 OOM / 无整机卡死）"
  exit 0
fi
exit 1
