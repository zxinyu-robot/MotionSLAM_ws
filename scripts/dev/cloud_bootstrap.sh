#!/usr/bin/env bash
# 云端容器内一次性引导：克隆云上所需第三方包并编译两个 workspace。
# 云上用 vkernel_sim 代替传感器+Super-LIO，所以不需要 livox/Super-LIO/PCT-python。
set -eo pipefail
cd /ws

UNITREE_COMMIT=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
SCAN_BRANCH=ros2-community

if [ ! -d src/unitree_ros2 ]; then
  git clone https://github.com/unitreerobotics/unitree_ros2.git src/unitree_ros2
fi
git -C src/unitree_ros2 checkout -q "${UNITREE_COMMIT}"

# 注：THIRD_PARTY.md 的 SCAN pin d62de08 是 Jetson 本地提交、上游不存在；
# 云端使用 ros2-community 分支头 + motionslam patch（已知 1 处
# closed_loop_controller.cpp hunk 与上游当前实现冲突，可安全跳过）。
mkdir -p scan_planner_ws/src
if [ ! -d scan_planner_ws/src/SCAN-Planner ]; then
  git clone -b "${SCAN_BRANCH}" https://github.com/wuyi2121/SCAN-Planner.git \
    scan_planner_ws/src/SCAN-Planner
fi
cd scan_planner_ws/src/SCAN-Planner
git checkout -q "${SCAN_BRANCH}"
if git apply --check /ws/patches/scan-planner-motionslam.patch 2>/dev/null; then
  git apply /ws/patches/scan-planner-motionslam.patch
elif ! git status --porcelain | grep -q '^.M\|^M'; then
  git apply --reject /ws/patches/scan-planner-motionslam.patch || true
fi
cd /ws

source /opt/ros/humble/setup.bash

# rosdep 补齐 SCAN-Planner 及主 workspace 依赖（cv_bridge、ros-gz 等）
rosdep init 2>/dev/null || true
rosdep update -q || true
rosdep install --from-paths scan_planner_ws/src src --ignore-src -y -q

(cd scan_planner_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)

# 主 workspace：只有自研 4 包（msgs/ipc/pipeline/bringup）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[cloud_bootstrap] done. 后续: source /ws/scripts/dev/source_cloud_env.sh"
