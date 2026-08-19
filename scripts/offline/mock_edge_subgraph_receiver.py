#!/usr/bin/env python3
"""边侧 mock：监听 :9877 收 sparse_subgraph.v1 JSON Lines."""
from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Optional


def run(host: str, port: int) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"mock edge subgraph receiver listening on {host}:{port}", flush=True)
    buf = b""
    count = 0
    while True:
        conn, addr = srv.accept()
        print(f"client connected: {addr}", flush=True)
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    count += 1
                    try:
                        obj = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError as exc:
                        print(f"[{count}] JSON error: {exc}", flush=True)
                        continue
                    schema = obj.get("schema", "?")
                    ver = obj.get("version", "?")
                    nodes = len(obj.get("nodes") or [])
                    frs = len(obj.get("frontiers") or [])
                    trig = obj.get("trigger_reason", "?")
                    print(
                        f"[{count}] {schema} v={ver} nodes={nodes} "
                        f"frontiers={frs} trigger={trig}",
                        flush=True,
                    )
        finally:
            conn.close()
            print(f"client disconnected: {addr}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Mock edge subgraph TCP receiver")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9877)
    args = parser.parse_args()
    try:
        run(args.host, args.port)
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
