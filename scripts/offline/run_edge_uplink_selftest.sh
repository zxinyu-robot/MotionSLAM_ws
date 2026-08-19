#!/usr/bin/env bash
# v1 边侧链路自测：mock 收 9877 JSON + 9878 MSRGB binary
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${HOST:-127.0.0.1}"
SUB_PORT="${SUB_PORT:-9877}"
RGB_PORT="${RGB_PORT:-9878}"

cleanup() {
  [[ -n "${RECV_PID:-}" ]] && kill "$RECV_PID" 2>/dev/null || true
}
trap cleanup EXIT

python3 "$ROOT/scripts/offline/mock_edge_uplink_receiver.py" \
  --host 0.0.0.0 --subgraph-port "$SUB_PORT" --rgb-port "$RGB_PORT" &
RECV_PID=$!
sleep 0.5

python3 - <<PY
import json
import socket
import sys

sys.path.insert(0, "$ROOT/src/motionslam_bringup/scripts")
from subgraph_snapshot import SubgraphBuildInput, build_sparse_subgraph_v1
from rgb_forward_frame import RgbForwardMetaInput, build_rgb_forward_meta, pack_rgb_forward_frame

def send_json(host, port, obj):
    line = json.dumps(obj, ensure_ascii=False) + "\n"
    s = socket.create_connection((host, port), timeout=2)
    s.sendall(line.encode("utf-8"))
    s.close()

snap = build_sparse_subgraph_v1(
    SubgraphBuildInput(
        version=1,
        session_id="selftest",
        tile_id="main",
        map_id="demo_live_map",
        trigger_reason="manual",
        robot_x=0.5,
        robot_y=0.1,
        robot_z=0.3,
        robot_yaw=0.0,
    )
)
send_json("$HOST", $SUB_PORT, snap)

payload = b"\\x00\\x00\\x00\\x01" + b"\\x00" * 64
meta = build_rgb_forward_meta(
    RgbForwardMetaInput(
        session_id="selftest",
        stream_field="video360p",
        payload=payload,
        time_frame=12345,
        seq=1,
    )
)
frame = pack_rgb_forward_frame(meta, payload)
s = socket.create_connection(("$HOST", $RGB_PORT), timeout=2)
s.sendall(frame)
s.close()
print("OK: sent subgraph JSON + RGB MSRGB binary selftest")
PY

sleep 0.5
echo "Done. Check mock receiver output above."
