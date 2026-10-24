#!/usr/bin/env bash
# 云端容器内环境：source /ws/scripts/dev/source_cloud_env.sh
source /opt/ros/humble/setup.bash
source /ws/scan_planner_ws/install/setup.bash
source /ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
