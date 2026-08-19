#!/usr/bin/env python3
"""向 GO2 directive_receiver (:9879) 注入 SemanticDirective 或 ActionGroup mock."""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import uuid


def build_directive(args: argparse.Namespace) -> dict:
    now = time.time_ns()
    return {
        "schema": "motionslam.semantic_directive.v1",
        "command_id": args.command_id or f"cmd_{uuid.uuid4().hex[:8]}",
        "trace_id": args.trace_id or "mock-trace",
        "session_id": args.session_id,
        "floor_id": args.floor_id,
        "map_id": args.map_id,
        "frame_id": args.frame_id,
        "map_version": args.map_version,
        "context_generation": args.context_generation,
        "issued_at_ns": now,
        "ttl_ms": args.ttl_ms,
        "language_query": args.query,
        "target_labels": [x.strip() for x in args.labels.split(",") if x.strip()],
        "search_mode": args.mode,
        "candidate_frontier_ids": [x.strip() for x in args.frontiers.split(",") if x.strip()],
        "confidence": args.confidence,
        "graph_bias": {
            "preferred_node_ids": [],
            "avoid_node_ids": [],
            "weights": {"geom": 0.5, "semantic": 0.5},
        },
    }


def build_action_group(args: argparse.Namespace) -> dict:
    now = time.time_ns()
    stages = []
    if args.mode == "search":
        stages.append(
            {
                "stage_id": "SEARCH_SEMANTIC_TARGET",
                "stage_type": "SEARCH",
                "waypoints": [],
                "timeout_ms": 180000,
                "allow_offline_continue": True,
            }
        )
    if args.nav_x is not None and args.nav_y is not None:
        stages.append(
            {
                "stage_id": "NAV_TO_DESTINATION",
                "stage_type": "NAVIGATE",
                "waypoints": [
                    {
                        "waypoint_id": "wp_001",
                        "frame_id": args.frame_id,
                        "x": args.nav_x,
                        "y": args.nav_y,
                        "z": args.nav_z,
                        "yaw_rad": args.nav_yaw,
                        "arrive_dist_m": 0.45,
                        "timeout_ms": 120000,
                    }
                ],
            }
        )
    stages.append(
        {
            "stage_id": "REPORT_RESULT",
            "stage_type": "REPORT",
            "waypoints": [],
        }
    )
    return {
        "schema": "motionslam.action_group.v1",
        "command_id": args.command_id or f"cmd_{uuid.uuid4().hex[:8]}",
        "trace_id": args.trace_id or "mock-trace",
        "task_type": "OBJNAV",
        "language_query": args.query,
        "session_id": args.session_id,
        "floor_id": args.floor_id,
        "map_id": args.map_id,
        "frame_id": args.frame_id,
        "map_version": args.map_version,
        "context_generation": args.context_generation,
        "created_ns": now,
        "ttl_ms": args.ttl_ms,
        "confidence": args.confidence,
        "stages": stages,
    }


def send_json_line(host: str, port: int, payload: dict) -> None:
    line = json.dumps(payload, ensure_ascii=False) + "\n"
    with socket.create_connection((host, port), timeout=5.0) as sock:
        sock.sendall(line.encode("utf-8"))
    print(f"sent to {host}:{port} schema={payload.get('schema')}", flush=True)
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="MVPI1 mock 下行 directive / action_group")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9879)
    parser.add_argument(
        "--kind",
        choices=("directive", "action_group"),
        default="directive",
        help="directive=SemanticDirective, action_group=ActionGroup",
    )
    parser.add_argument("--mode", choices=("explore", "navigate", "search"), default="explore")
    parser.add_argument("--query", default="找红色灭火器")
    parser.add_argument("--labels", default="fire_extinguisher,extinguisher")
    parser.add_argument("--frontiers", default="bt_next_subgoal")
    parser.add_argument("--session-id", default="demo_live")
    parser.add_argument("--floor-id", default="floor_01")
    parser.add_argument("--map-id", default="demo_live_map")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--map-version", type=int, default=1)
    parser.add_argument("--context-generation", type=int, default=0)
    parser.add_argument("--ttl-ms", type=int, default=30000)
    parser.add_argument("--confidence", type=float, default=0.75)
    parser.add_argument("--command-id", default="")
    parser.add_argument("--trace-id", default="")
    parser.add_argument("--nav-x", type=float, default=None)
    parser.add_argument("--nav-y", type=float, default=None)
    parser.add_argument("--nav-z", type=float, default=0.35)
    parser.add_argument("--nav-yaw", type=float, default=0.0)
    parser.add_argument("--bad-session", action="store_true", help="注入错误 session_id 测 REJECT")
    parser.add_argument("--expired", action="store_true", help="注入过期 created_ns 测 REJECT_EXPIRED")
    args = parser.parse_args()

    if args.bad_session:
        args.session_id = "wrong_session"

    if args.kind == "action_group":
        payload = build_action_group(args)
    else:
        payload = build_directive(args)

    if args.expired:
        payload["created_ns"] = time.time_ns() - 60_000_000_000
        payload["ttl_ms"] = 1000
        if "issued_at_ns" in payload:
            payload["issued_at_ns"] = payload["created_ns"]

    try:
        send_json_line(args.host, args.port, payload)
    except OSError as exc:
        print(f"ERROR: connect/send failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
