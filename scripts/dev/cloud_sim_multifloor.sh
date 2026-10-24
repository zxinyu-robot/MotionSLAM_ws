#!/usr/bin/env bash
# 云端多楼层闭环：一楼→坡道→二楼巡回。容器内执行。
set -eo pipefail
source /ws/scripts/dev/source_cloud_env.sh

exec ros2 launch motionslam_bringup demo_scan_stack.launch.py \
  with_sensors:=false with_lio:=false with_mock_robot:=true \
  with_glass_aware:=false with_foxglove:=false with_edge_uplink:=false \
  vkernel_params_file:=/ws/src/motionslam_bringup/config/vkernel_multifloor.yaml \
  waypoints_file:=/ws/src/motionslam_bringup/config/demo_scan_waypoints_multifloor.yaml \
  extra_params_file:=/ws/src/motionslam_bringup/config/demo_scan_planner_cloud.yaml \
  demo_profile:=demo_short autostart_lifecycle:=true
