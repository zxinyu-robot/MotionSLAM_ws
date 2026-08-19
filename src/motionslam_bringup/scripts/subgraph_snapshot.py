#!/usr/bin/env python3
"""组装 motionslam.sparse_subgraph.v1 JSON（纯函数，便于单测）."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Optional

SCHEMA = "motionslam.sparse_subgraph.v1"
LAYER_ID = "topology_subgraph"


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_dict(x: float, y: float, z: float, yaw: float) -> dict[str, float]:
    return {
        "x": round(x, 4),
        "y": round(y, 4),
        "z": round(z, 4),
        "yaw_deg": round(math.degrees(yaw), 2),
    }


@dataclass
class KeyframeView:
    keyframe_id: int
    x: float
    y: float
    z: float
    yaw: float
    timestamp_ns: int = 0


@dataclass
class FrontierView:
    frontier_id: str
    x: float
    y: float
    z: float
    geom_score: float = 1.0


@dataclass
class SubgraphBuildInput:
    version: int
    session_id: str
    tile_id: str
    map_id: str
    trigger_reason: str
    robot_x: float
    robot_y: float
    robot_z: float
    robot_yaw: float
    floor_id: str = "floor_01"
    keyframes: list[KeyframeView] = field(default_factory=list)
    frontiers: list[FrontierView] = field(default_factory=list)
    context_generation: int = 0
    pgo_state: str = "IDLE"
    timestamp_ns: Optional[int] = None


def build_sparse_subgraph_v1(data: SubgraphBuildInput) -> dict[str, Any]:
    ts = data.timestamp_ns if data.timestamp_ns is not None else time.time_ns()

    nodes: list[dict[str, Any]] = []
    edges: list[dict[str, str]] = []
    keyframes_meta: list[dict[str, Any]] = []

    if data.keyframes:
        for kf in data.keyframes:
            node_id = f"kf_{kf.keyframe_id}"
            rgb_ref = f"rgb_{kf.keyframe_id}"
            nodes.append(
                {
                    "node_id": node_id,
                    "x": round(kf.x, 4),
                    "y": round(kf.y, 4),
                    "z": round(kf.z, 4),
                    "rgb_keyframe_ref": rgb_ref,
                }
            )
            keyframes_meta.append(
                {
                    "keyframe_id": kf.keyframe_id,
                    "timestamp_ns": kf.timestamp_ns,
                    "pose": pose_dict(kf.x, kf.y, kf.z, kf.yaw),
                }
            )
        for i in range(len(data.keyframes) - 1):
            edges.append(
                {
                    "from": f"kf_{data.keyframes[i].keyframe_id}",
                    "to": f"kf_{data.keyframes[i + 1].keyframe_id}",
                }
            )
    else:
        nodes.append(
            {
                "node_id": "robot",
                "x": round(data.robot_x, 4),
                "y": round(data.robot_y, 4),
                "z": round(data.robot_z, 4),
                "rgb_keyframe_ref": "rgb_robot",
            }
        )

    frontiers = [
        {
            "frontier_id": f.frontier_id,
            "x": round(f.x, 4),
            "y": round(f.y, 4),
            "z": round(f.z, 4),
            "geom_score": round(f.geom_score, 4),
        }
        for f in data.frontiers
    ]

    return {
        "schema": SCHEMA,
        "layer_id": LAYER_ID,
        "version": data.version,
        "session_id": data.session_id,
        "floor_id": data.floor_id,
        "tile_id": data.tile_id,
        "map_id": data.map_id,
        "timestamp_ns": ts,
        "trigger_reason": data.trigger_reason,
        "robot_pose": pose_dict(data.robot_x, data.robot_y, data.robot_z, data.robot_yaw),
        "nodes": nodes,
        "edges": edges,
        "frontiers": frontiers,
        "keyframes_meta": keyframes_meta,
        "context_generation": data.context_generation,
        "pgo_state": data.pgo_state,
    }
