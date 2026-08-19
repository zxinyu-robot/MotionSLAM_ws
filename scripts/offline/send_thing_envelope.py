#!/usr/bin/env python3
"""向狗端 :9879 发送 motionslam.thing_envelope.v1（一行 JSON）。"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import uuid
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description="Send thing_envelope.v1 to directive_receiver")
    p.add_argument("--host", default="127.0.0.1", help="GO2 Orin IP")
    p.add_argument("--port", type=int, default=9879)
    p.add_argument(
        "--file",
        default=str(Path(__file__).with_name("thing_envelope_qwen_navigate.json")),
    )
    p.add_argument("--x", type=float, default=None, help="override goal x")
    p.add_argument("--y", type=float, default=None, help="override goal y")
    p.add_argument("--z", type=float, default=None, help="override goal z")
    args = p.parse_args()

    raw = json.loads(Path(args.file).read_text(encoding="utf-8"))
    raw["tid"] = f"qwen-{uuid.uuid4().hex[:8]}"
    raw["timestamp"] = time.time_ns()
    raw["ttl_ms"] = int(raw.get("ttl_ms") or 30000)
    goal = raw["data"]["services"]["actions"][0]["goal"]
    if args.x is not None:
        goal["x"] = args.x
    if args.y is not None:
        goal["y"] = args.y
    if args.z is not None:
        goal["z"] = args.z

    line = json.dumps(raw, ensure_ascii=False) + "\n"
    with socket.create_connection((args.host, args.port), timeout=5) as sock:
        sock.sendall(line.encode("utf-8"))
    print(f"sent tid={raw['tid']} goal=({goal['x']},{goal['y']},{goal['z']}) → {args.host}:{args.port}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
