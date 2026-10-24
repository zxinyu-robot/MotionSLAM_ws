#!/usr/bin/env python3
"""Task Intent 云端客户端（无 ROS 依赖）：发 intent → :9879，收 feedback ← :9880。

用法:
  python3 task_intent_client.py --host 127.0.0.1 --goal 2.0 0.5 0.35
  python3 task_intent_client.py --goal 2.0 0.5 0.35 --return-home --timeout 240
退出码: 0=任务 SUCCEEDED；1=FAILED/REJECTED；2=超时。
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time

RECEIVED = []
DONE = threading.Event()
TERMINAL_RC = 2


def _reader(host: str, port: int) -> None:
    global TERMINAL_RC
    while not DONE.is_set():
        try:
            with socket.create_connection((host, port), timeout=5) as conn:
                f = conn.makefile("r", encoding="utf-8")
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    RECEIVED.append(msg)
                    print(
                        f"[fb] {msg.get('event')} state={msg.get('state')} "
                        f"detail={msg.get('detail')}",
                        flush=True,
                    )
                    if msg.get("event") == "task_terminal":
                        TERMINAL_RC = 0 if msg.get("state") == "succeeded" else 1
                        DONE.set()
                        return
        except OSError:
            time.sleep(0.5)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--downlink-port", type=int, default=9879)
    ap.add_argument("--uplink-port", type=int, default=9880)
    ap.add_argument("--goal", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    default=[2.0, 0.5, 0.35])
    ap.add_argument("--return-home", action="store_true")
    ap.add_argument("--map-id", default="demo_live_map")
    ap.add_argument("--revision", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=240.0)
    args = ap.parse_args()

    t = threading.Thread(target=_reader, args=(args.host, args.uplink_port), daemon=True)
    t.start()
    time.sleep(0.5)  # 让 feedback 连接先建立

    skills = [{"name": "Navigate", "goal": {"x": args.goal[0], "y": args.goal[1], "z": args.goal[2]}}]
    if args.return_home:
        skills.append({"name": "ReturnHome"})
    intent = {
        "schema": "motionslam.task_intent.v1",
        "task_id": f"cloud-task-{int(time.time())}",
        "robot": {"platform": "unitree_go2_edu", "sn": "go2_001"},
        "world_ref": {
            "map_id": args.map_id,
            "floor_id": "floor_01",
            "frame_id": "world",
            "revision": args.revision,
        },
        "skills": skills,
        "policy": {"deadline_s": args.timeout},
    }
    print(f"[cli] send task_intent: {json.dumps(intent['skills'])}", flush=True)
    with socket.create_connection((args.host, args.downlink_port), timeout=10) as conn:
        conn.sendall((json.dumps(intent, ensure_ascii=False) + "\n").encode("utf-8"))
        f = conn.makefile("r", encoding="utf-8")
        reply = json.loads(f.readline())
        print(f"[cli] downlink reply: {reply}", flush=True)
        if reply.get("type") == "task_rejected":
            return 1

    DONE.wait(timeout=args.timeout + 10)
    return TERMINAL_RC if DONE.is_set() else 2


if __name__ == "__main__":
    sys.exit(main())
