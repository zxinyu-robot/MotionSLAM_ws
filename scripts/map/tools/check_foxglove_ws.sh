#!/usr/bin/env bash
# 检查 foxglove_bridge WebSocket 握手 (8765=sdk.v1, 8766=websocket.v1)
set -euo pipefail

HOST="${1:-127.0.0.1}"

python3 - "$HOST" <<'PY'
import socket, base64, sys

host = sys.argv[1]

def probe(port, subproto):
    key = base64.b64encode(b"probe").decode()
    lines = [
        f"GET / HTTP/1.1",
        f"Host: {host}:{port}",
        "Upgrade: websocket",
        "Connection: Upgrade",
        f"Sec-WebSocket-Key: {key}",
        "Sec-WebSocket-Version: 13",
    ]
    if subproto:
        lines.append(f"Sec-WebSocket-Protocol: {subproto}")
    req = "\r\n".join(lines) + "\r\n\r\n"
    try:
        s = socket.create_connection((host, port), timeout=3)
        s.sendall(req.encode())
        resp = s.recv(256).decode(errors="replace")
        s.close()
        status = resp.split("\r\n", 1)[0]
        ok = "101 Switching Protocols" in status
        return ok, status
    except OSError as e:
        return False, str(e)

print(f"Foxglove bridge 探测: {host}")
for port, sub, label in [
    (8765, "foxglove.sdk.v1", "新版 Foxglove Studio / app.foxglove.dev"),
    (8766, "foxglove.websocket.v1", "旧版 Foxglove / Lichtblick"),
]:
    ok, status = probe(port, sub)
    mark = "OK" if ok else "FAIL"
    print(f"  [{mark}] :{port} ({sub}) -> {status}")
    if ok:
        print(f"       连接 URL: ws://{host}:{port}")

print()
print("若 8765 FAIL 且 8766 OK: 开发机请连 ws://<IP>:8766")
print("若 8765 OK: 请用最新 Foxglove 连 ws://<IP>:8765")
PY
