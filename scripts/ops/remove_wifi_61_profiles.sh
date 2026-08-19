#!/usr/bin/env bash
# 移除含固定 IP ${GO2_WLAN_IP} 及重复的 5G WiFi 配置，保留 moushen.ai（DHCP，当前 .127）
set -euo pipefail

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

CONN_DIR=/etc/NetworkManager/system-connections
FILES=(
  "moushen.ai-5G.nmconnection"
  "moushen.ai-5G 1.nmconnection"
  "moushen5g.nmconnection"
)

for f in "${FILES[@]}"; do
  path="${CONN_DIR}/${f}"
  if [[ -f "$path" ]]; then
    rm -f "$path"
    echo "Deleted: $path"
  else
    echo "Skip (not found): $path"
  fi
done

nmcli general reload
echo ""
echo "=== Remaining connections ==="
nmcli connection show
echo ""
echo "=== wlan0 ==="
ip -br addr show wlan0
