#!/usr/bin/env bash
# PCT-Planner native .so 运行时环境（供 launch / 冒烟脚本 source）
set -euo pipefail

_ws="${MOTIONSLAM_WS:-/ws}"
_lib="${PCT_PLANNER_PLANNER_LIB:-${_ws}/src/PCT-Planner/pct_planner/planner/lib}"
_gtsam="${_lib}/3rdparty/gtsam-4.1.1/install/lib"
_smooth="${_lib}/build/src/common/smoothing"

export LD_LIBRARY_PATH="${_gtsam}:${_smooth}:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${_lib}:${PYTHONPATH:-}"
