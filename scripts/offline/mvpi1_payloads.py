#!/usr/bin/env python3
"""MVPI1 下行 payload 构造（mock / edge planner 共用）."""
from __future__ import annotations

import time
import uuid
from typing import Any, Optional


def new_command_id(prefix: str = "cmd") -> str:
    return f"{prefix}_{uuid.uuid4().hex[:8]}"


def build_semantic_directive(
    *,
    query: str = "找红色灭火器",
    session_id: str = "demo_live",
    floor_id: str = "floor_01",
    map_id: str = "demo_live_map",
    frame_id: str = "world",
    map_version: int = 0,
    context_generation: int = 0,
    ttl_ms: int = 30000,
    search_mode: str = "explore",
    candidate_frontier_ids: Optional[list[str]] = None,
    confidence: float = 0.62,
    command_id: str = "",
    trace_id: str = "mock-trace",
    preferred_node_ids: Optional[list[str]] = None,
    geom_weight: float = 0.5,
    semantic_weight: float = 0.5,
) -> dict[str, Any]:
    now = time.time_ns()
    frontier_ids = candidate_frontier_ids or []
    return {
        "schema": "motionslam.semantic_directive.v1",
        "command_id": command_id or new_command_id("dir"),
        "trace_id": trace_id,
        "session_id": session_id,
        "floor_id": floor_id,
        "map_id": map_id,
        "frame_id": frame_id,
        "map_version": map_version,
        "context_generation": context_generation,
        "issued_at_ns": now,
        "ttl_ms": ttl_ms,
        "language_query": query,
        "target_labels": [],
        "search_mode": search_mode,
        "candidate_frontier_ids": frontier_ids,
        "graph_bias": {
            "preferred_node_ids": preferred_node_ids or [],
            "avoid_node_ids": [],
            "weights": {"geom": geom_weight, "semantic": semantic_weight},
        },
        "confidence": confidence,
    }


def build_action_group(
    *,
    query: str = "找红色灭火器",
    session_id: str = "demo_live",
    floor_id: str = "floor_01",
    map_id: str = "demo_live_map",
    frame_id: str = "world",
    map_version: int = 0,
    context_generation: int = 0,
    ttl_ms: int = 30000,
    confidence: float = 0.75,
    command_id: str = "",
    trace_id: str = "mock-trace",
    include_search: bool = False,
    include_nav: bool = False,
    nav_x: Optional[float] = None,
    nav_y: Optional[float] = None,
    nav_z: float = 0.35,
    nav_yaw: float = 0.0,
) -> dict[str, Any]:
    now = time.time_ns()
    stages: list[dict[str, Any]] = []
    if include_search:
        stages.append(
            {
                "stage_id": "SEARCH_SEMANTIC_TARGET",
                "stage_type": "SEARCH",
                "waypoints": [],
                "timeout_ms": 180000,
                "allow_offline_continue": True,
            }
        )
    if include_nav and nav_x is not None and nav_y is not None:
        stages.append(
            {
                "stage_id": "NAV_TO_DESTINATION",
                "stage_type": "NAVIGATE",
                "waypoints": [
                    {
                        "waypoint_id": "wp_001",
                        "frame_id": frame_id,
                        "x": nav_x,
                        "y": nav_y,
                        "z": nav_z,
                        "yaw_rad": nav_yaw,
                        "arrive_dist_m": 0.45,
                        "timeout_ms": 120000,
                        "allow_offline_continue": True,
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
        "command_id": command_id or new_command_id("ag"),
        "trace_id": trace_id,
        "task_type": "OBJNAV",
        "language_query": query,
        "session_id": session_id,
        "floor_id": floor_id,
        "map_id": map_id,
        "frame_id": frame_id,
        "map_version": map_version,
        "context_generation": context_generation,
        "created_ns": now,
        "ttl_ms": ttl_ms,
        "confidence": confidence,
        "stages": stages,
    }


def send_json_line(host: str, port: int, payload: dict[str, Any]) -> None:
    import json
    import socket

    line = json.dumps(payload, ensure_ascii=False) + "\n"
    with socket.create_connection((host, port), timeout=5.0) as sock:
        sock.sendall(line.encode("utf-8"))
