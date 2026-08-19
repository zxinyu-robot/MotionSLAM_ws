#!/usr/bin/env bash
# 宿主机一键 H5：启 demo_scan_stack + verify_scan_planner_h5
# 由 motionslam verify h5-docker 调用，勿直接改路径引用
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"

DISTANCE="${1:-3}"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DOCKER="/ws/logs/scan_planner_h5_${TS}.log"
RESULT_HOST="$ROOT/logs/scan_planner_h5_result_${TS}.txt"
mkdir -p "$ROOT/logs"

if ! docker ps --format '{{.Names}}' | grep -qx motionslam; then
  echo "请先: ./docker/run_container.sh"
  exit 1
fi

echo "[1/5] 停旧栈..."
docker exec motionslam bash -lc 'bash /ws/scripts/nav/stop_nav.sh' || true
sleep 1

echo "[2/5] build MotionCommand 链路 + bringup..."
docker exec motionslam bash -lc '
  cd /ws && source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select motionslam_msgs motionslam_pipeline motionslam_bringup
  cd /ws/scan_planner_ws && source /ws/install/setup.bash
  colcon build --symlink-install --packages-select scan_planner
  test -f /ws/scan_planner_ws/install/setup.bash
'

echo "[3/5] 后台启动 demo_scan_stack..."
docker exec motionslam bash -lc "
  source /opt/ros/humble/setup.bash
  source /ws/scripts/dev/source_ws_env.sh
  nohup ros2 launch motionslam_bringup demo_scan_stack.launch.py \
    with_foxglove:=false with_pose_graph:=false autostart_lifecycle:=false \
    > $LOG_DOCKER 2>&1 &
  echo \$! > /tmp/demo_scan_stack.pid
"

echo "[4/5] 等待栈就绪..."
docker exec motionslam bash -lc '
  source /ws/scripts/dev/source_ws_env.sh
  for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q scan_planner_node; then
      if timeout 3 ros2 topic echo /lio/robo/odom --once >/dev/null 2>&1; then
        echo "OK: stack ready (${i}s)"
        sleep 8
        exit 0
      fi
    fi
    sleep 2
  done
  echo "ERROR: 栈未就绪, 见 '"$LOG_DOCKER"'"
  tail -30 '"$LOG_DOCKER"' || true
  exit 1
'

echo "[5/5] H5 distance=${DISTANCE}m (MotionCommand 链路)..."
set +e
docker exec motionslam bash -lc "
  source /ws/scripts/dev/source_ws_env.sh
  python3 /ws/scripts/verify/verify_scan_planner_h5.py --distance $DISTANCE --segment-m 1.5 --timeout 240
" 2>&1 | tee "$RESULT_HOST"
RC=${PIPESTATUS[0]}
set -e
echo "结果: $RESULT_HOST (exit=$RC)"
exit $RC
