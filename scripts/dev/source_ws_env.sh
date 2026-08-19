#!/usr/bin/env bash
# MotionSLAM 栈统一环境：ROS humble + MotionSLAM + scan_planner + pct_planner
set +u
source /opt/ros/humble/setup.bash
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /ws/install/setup.bash
if [[ -f /ws/scan_planner_ws/install/setup.bash ]]; then
  source /ws/scan_planner_ws/install/setup.bash
fi
if [[ -f /ws/pct_planner_ws/install/setup.bash ]]; then
  source /ws/pct_planner_ws/install/setup.bash
fi
if [[ -f /ws/src/motionslam_bringup/scripts/pct_planner_env.sh ]]; then
  # shellcheck source=/dev/null
  source /ws/src/motionslam_bringup/scripts/pct_planner_env.sh
fi
