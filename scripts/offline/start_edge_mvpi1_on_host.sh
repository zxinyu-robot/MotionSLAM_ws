#!/usr/bin/env bash
# 在边侧机器（如 ubuntu@${EDGE_HOST}）上启动 MVPI1 mock planner。
# GO2 侧 edge_uplink 需指向本机 IP（EDGE_HOST=${EDGE_HOST}）。
#
# 用法（在 ${EDGE_HOST} 上）:
#   GO2_HOST=${GO2_IP} bash start_edge_mvpi1_on_host.sh
#   GO2_HOST=${GO2_IP} bash start_edge_mvpi1_on_host.sh --start-search
set -euo pipefail

GO2_HOST="${GO2_HOST:-${GO2_IP}}"
QUERY="${QUERY:-找红色灭火器}"
LISTEN_HOST="${LISTEN_HOST:-0.0.0.0}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Edge MVPI1 planner @ $(hostname -I | awk '{print $1}') ==="
echo "  listen: ${LISTEN_HOST}:9877/9878/9880"
echo "  go2 directive push: ${GO2_HOST}:9879"
echo ""

exec python3 "$DIR/edge_mvpi1_planner.py" \
  --host "$LISTEN_HOST" \
  --go2-host "$GO2_HOST" \
  --query "$QUERY" \
  "$@"
