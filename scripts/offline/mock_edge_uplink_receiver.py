#!/usr/bin/env python3
"""边侧 mock：9877 子图 JSON Lines；9878 RGB 二进制 MSRGB 帧."""
from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from typing import Callable, Optional

ROOT = str(__import__("pathlib").Path(__file__).resolve().parents[2])
sys.path.insert(0, f"{ROOT}/src/motionslam_bringup/scripts")
from rgb_forward_frame import unpack_rgb_forward_frame  # noqa: E402
from semantic_bev_frame import unpack_semantic_token_frame  # noqa: E402


def _serve_json_lines(
    name: str,
    host: str,
    port: int,
    on_line: Optional[Callable[[str, int, dict], None]] = None,
) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"[{name}] listening on {host}:{port}", flush=True)
    count = 0
    buf = b""
    while True:
        conn, addr = srv.accept()
        print(f"[{name}] client {addr}", flush=True)
        try:
            while True:
                chunk = conn.recv(65536)
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
                        print(f"[{name}][{count}] JSON error: {exc}", flush=True)
                        continue
                    if on_line:
                        on_line(name, count, obj)
        finally:
            conn.close()


def _serve_rgb_binary(host: str, port: int) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"[rgb] listening on {host}:{port} (MSRGB binary)", flush=True)
    count = 0
    while True:
        conn, addr = srv.accept()
        print(f"[rgb] client {addr}", flush=True)
        buf = b""
        try:
            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 14:
                    try:
                        meta, payload, consumed = unpack_rgb_forward_frame(buf)
                    except ValueError:
                        break
                    buf = buf[consumed:]
                    count += 1
                    print(
                        f"[rgb][{count}] {meta.get('schema')} seq={meta.get('seq')} "
                        f"field={meta.get('stream_field')} bytes={len(payload)} "
                        f"time_frame={meta.get('time_frame')}",
                        flush=True,
                    )
        finally:
            conn.close()


def _serve_token_binary(host: str, port: int) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"[token] listening on {host}:{port} (MSTOK binary)", flush=True)
    count = 0
    while True:
        conn, addr = srv.accept()
        print(f"[token] client {addr}", flush=True)
        buf = b""
        try:
            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 14:
                    try:
                        meta, payload, consumed = unpack_semantic_token_frame(buf)
                    except ValueError:
                        break
                    buf = buf[consumed:]
                    count += 1
                    grid = meta.get("grid") or {}
                    print(
                        f"[token][{count}] {meta.get('schema')} seq={meta.get('seq')} "
                        f"grid={grid.get('width')}x{grid.get('height')} "
                        f"bytes={len(payload)} gen={meta.get('context_generation')}",
                        flush=True,
                    )
        finally:
            conn.close()


def _print_subgraph(_name: str, count: int, obj: dict) -> None:
    print(
        f"[subgraph][{count}] {obj.get('schema')} v={obj.get('version')} "
        f"nodes={len(obj.get('nodes') or [])} frontiers={len(obj.get('frontiers') or [])} "
        f"trigger={obj.get('trigger_reason')}",
        flush=True,
    )


def send_directive(host: str, port: int, payload: dict) -> None:
    line = json.dumps(payload, ensure_ascii=False) + "\n"
    conn = socket.create_connection((host, port), timeout=2.0)
    conn.sendall(line.encode("utf-8"))
    conn.close()
    print(f"[directive] sent to {host}:{port}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Mock edge uplink receiver (9876/9877/9878)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--token-port", type=int, default=9876)
    parser.add_argument("--subgraph-port", type=int, default=9877)
    parser.add_argument("--rgb-port", type=int, default=9878)
    parser.add_argument("--inject-directive", action="store_true")
    parser.add_argument("--directive-host", default="127.0.0.1")
    parser.add_argument("--directive-port", type=int, default=9879)
    args = parser.parse_args()

    t1 = threading.Thread(
        target=_serve_json_lines,
        args=("subgraph", args.host, args.subgraph_port, _print_subgraph),
        daemon=True,
    )
    t2 = threading.Thread(
        target=_serve_rgb_binary,
        args=(args.host, args.rgb_port),
        daemon=True,
    )
    t3 = threading.Thread(
        target=_serve_token_binary,
        args=(args.host, args.token_port),
        daemon=True,
    )
    t1.start()
    t2.start()
    t3.start()

    if args.inject_directive:
        time.sleep(0.5)
        send_directive(
            args.directive_host,
            args.directive_port,
            {
                "schema": "motionslam.semantic_directive.v1",
                "session_id": "demo_live",
                "map_version": 1,
                "issued_at_ns": time.time_ns(),
                "ttl_ms": 30000,
                "candidate_frontier_ids": ["bt_next_subgoal"],
                "confidence": 0.9,
                "instruction": "mock_edge_directive",
            },
        )

    try:
        t1.join()
        t2.join()
        t3.join()
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
