#!/usr/bin/env bash
# 云端虚拟设备闭环：vkernel（虚拟雷达/里程计）+ PCT→BT→SCAN→forwarder（+可选 Thin Harness）
# 容器内执行：bash /ws/scripts/dev/cloud_sim.sh [with_thin_harness]
set -eo pipefail
source /ws/scripts/dev/source_cloud_env.sh

THIN="${1:-false}"
SEMANTIC="false"
if [ "${THIN}" = "true" ]; then SEMANTIC="true"; fi
echo "[cloud_sim] sensors=off lio=off mock_robot=on thin_harness=${THIN} semantic=${SEMANTIC}"
exec ros2 launch motionslam_bringup demo_scan_stack.launch.py \
  with_sensors:=false with_lio:=false with_mock_robot:=true \
  with_thin_harness:="${THIN}" with_semantic_objnav:="${SEMANTIC}" \
  with_glass_aware:=false with_foxglove:=false with_edge_uplink:=false \
  extra_params_file:=/ws/src/motionslam_bringup/config/demo_scan_planner_cloud.yaml \
  demo_profile:=demo_short autostart_lifecycle:=true
