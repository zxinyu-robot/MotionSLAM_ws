#!/usr/bin/env bash
# 端侧 apt 换清华源（Ubuntu 22.04 jammy + ROS2 Humble）
# 容器内: docker exec motionslam bash -lc '/ws/scripts/ops/setup_apt_tsinghua.sh'
# 宿主机: sudo ./scripts/ops/setup_apt_tsinghua.sh
set -euo pipefail

ARCH="$(dpkg --print-architecture)"
CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME:-jammy}")"
MIRROR="https://mirrors.tuna.tsinghua.edu.cn"

echo "=== apt 换清华源 (${CODENAME} ${ARCH}) ==="

if [[ "$(id -u)" -ne 0 ]]; then
  exec sudo bash "$0" "$@"
fi

backup() {
  local f="$1"
  [[ -f "$f" && ! -f "${f}.bak" ]] && cp -a "$f" "${f}.bak"
}

if [[ "$ARCH" == "arm64" || "$ARCH" == "armhf" ]]; then
  UBUNTU_PATH="${MIRROR}/ubuntu-ports"
else
  UBUNTU_PATH="${MIRROR}/ubuntu"
fi

backup /etc/apt/sources.list
cat > /etc/apt/sources.list <<EOF
deb ${UBUNTU_PATH}/ ${CODENAME} main restricted universe multiverse
deb ${UBUNTU_PATH}/ ${CODENAME}-updates main restricted universe multiverse
deb ${UBUNTU_PATH}/ ${CODENAME}-backports main restricted universe multiverse
deb ${UBUNTU_PATH}/ ${CODENAME}-security main restricted universe multiverse
EOF

backup /etc/apt/sources.list.d/ros2.list
cat > /etc/apt/sources.list.d/ros2.list <<EOF
deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${MIRROR}/ros2/ubuntu ${CODENAME} main
EOF

apt-get clean
apt-get update -qq
echo "OK: 清华源已生效 ($(grep -m1 '^deb ' /etc/apt/sources.list))"
