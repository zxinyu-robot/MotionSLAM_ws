#!/usr/bin/env bash
# 云端多楼层闭环：一楼→坡道→二楼巡回。容器内执行。
# WITH_EDGE=1 时同时启动 mock 边端接收端并打开边端上行（云端化验证）。
set -eo pipefail
source /ws/scripts/dev/source_cloud_env.sh

EDGE_ARGS="with_edge_uplink:=false"
if [[ "${WITH_EDGE:-0}" == "1" ]]; then
  # 边端上行云端化：本地 mock 接收端（token/subgraph/rgb 三通道）
  nohup python3 /ws/scripts/offline/mock_edge_uplink_receiver.py \
    > /tmp/mock_edge_receiver.log 2>&1 &
  sleep 1
  EDGE_ARGS="with_edge_uplink:=true edge_host:=127.0.0.1"
fi

exec ros2 launch motionslam_bringup demo_scan_stack.launch.py \
  with_sensors:=false with_lio:=false with_mock_robot:=true \
  with_glass_aware:=false with_foxglove:=false ${EDGE_ARGS} \
  vkernel_params_file:=/ws/src/motionslam_bringup/config/vkernel_multifloor.yaml \
  waypoints_file:=/ws/src/motionslam_bringup/config/demo_scan_waypoints_multifloor.yaml \
  extra_params_file:=/ws/src/motionslam_bringup/config/demo_scan_planner_cloud.yaml \
  demo_profile:=demo_short autostart_lifecycle:=true
