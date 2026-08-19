#!/usr/bin/env bash
# 容器内编译整个 colcon 工作区 (在 /ws 下执行)
# 用法: ./scripts/dev/build_ws.sh [colcon 额外参数]
set -eo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPTS/.."

# 系统时间错误会导致 apt InRelease / gmake mtime 异常 (常见: 时钟停在 1970)
YEAR="$(date +%Y)"
if [[ "$YEAR" -lt 2020 ]]; then
  echo "ERROR: 系统时间异常 (date => $(date))"
  echo "  容器内无 systemd，不要用 timedatectl；本容器 --privileged 可试:"
  echo "    date -s \"\$(date -u +%Y-%m-%d\ %H:%M:%S)\"   # 或手动写 2026-07-13 15:00:00"
  echo "  推荐在宿主机 (退出容器后) 修正并开 NTP:"
  echo "    sudo timedatectl set-ntp false"
  echo "    sudo timedatectl set-time \"2026-07-13 15:00:00\""
  echo "    sudo timedatectl set-ntp true"
  echo "  详见 docs/运维/runbook.md §3"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -e

# unitree_api / unitree_go / unitree_hg 需要 CycloneDDS IDL 生成器
UNITREE_DDS_DEB=ros-humble-rosidl-generator-dds-idl
if ! dpkg-query -W -f='${Status}' "$UNITREE_DDS_DEB" 2>/dev/null | grep -q "install ok installed"; then
  echo "WARN: 缺少 $UNITREE_DDS_DEB (unitree_api 编译需要)"
  echo "  临时修复 (容器内): apt-get update && apt-get install -y $UNITREE_DDS_DEB"
  echo "  永久修复 (宿主机):  docker build -t motionslam:humble docker/  后重建容器"
  if [[ "${MOTIONSLAM_AUTO_APT:-0}" == "1" ]]; then
    apt-get update && apt-get install -y "$UNITREE_DDS_DEB"
  else
    exit 1
  fi
fi

# NOTE: livox_ros_driver2 用版本化的 package 文件, colcon 前必须替换 (参考其 build.sh)
LIVOX=src/livox_ros_driver2
if [ ! -f "$LIVOX/package.xml" ] || ! grep -q ament_cmake "$LIVOX/package.xml"; then
    cp -f "$LIVOX/package_ROS2.xml" "$LIVOX/package.xml"
fi
[ -d "$LIVOX/launch" ] || cp -rf "$LIVOX/launch_ROS2" "$LIVOX/launch"

# SCAN-Planner 仅在 scan_planner_ws 独立工作区构建，主仓 src 扫描时跳过同名包。
# PCT-Planner 仅在 pct_planner_ws 独立工作区构建。
# ego-planner-swarm 仅在 feature/hybrid-baseline 构建；dev 跳过整包。
SCAN_PLANNER_PACKAGES=(
  plan_env path_searching bspline_opt scan_planner scan_planner_msgs traj_utils
  go2_description local_sensing_node map_generator mockamap
  odom_visualization pose_utils waypoint_generator
)

PCT_PLANNER_PACKAGES=(pct_planner)

EGO_PLANNER_PACKAGES=(
  plan_manage drone_detect rosmsg_tcp_bridge
  mockamap map_generator so3_quadrotor_simulator so3_control fake_drone
  local_sensing cmake_utils quadrotor_msgs pose_utils multi_map_server
  odom_visualization uav_utils waypoint_generator
)

IGNORE_PACKAGES=("${SCAN_PLANNER_PACKAGES[@]}" "${PCT_PLANNER_PACKAGES[@]}" "${EGO_PLANNER_PACKAGES[@]}")

colcon build --symlink-install \
    --base-paths src \
    --packages-ignore "${IGNORE_PACKAGES[@]}" \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release \
    "$@"

echo "OK: source install/setup.bash 后可用"
