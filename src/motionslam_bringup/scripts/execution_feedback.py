#!/usr/bin/env python3
"""任务 B：边端指令安全执行反馈 payload（纯函数）."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from data_layer_types import DataLayerId

SCHEMA = "motionslam.execution_feedback.v1"


@dataclass
class ExecutionFeedbackInput:
    session_id: str
    floor_id: str
    tile_id: str
    map_id: str
    context_generation: int
    command_id: str = ""
    trace_id: str = ""
    phase: str = ""
    event: str = ""
    accepted: bool = True
    reason: str = ""
    safety_state: str = "OK"
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_z: float = 0.0
    robot_yaw_deg: float = 0.0
    layer_version: int = 0
    timestamp_ns: Optional[int] = None
    extra: Optional[dict[str, Any]] = None
    mode_id: str = ""
    mode_state: str = ""
    scene_type: str = ""
    events: Optional[list[dict[str, Any]]] = None
    world_fragments: Optional[list[dict[str, Any]]] = None


def build_execution_feedback_v1(data: ExecutionFeedbackInput) -> dict[str, Any]:
    ts = data.timestamp_ns if data.timestamp_ns is not None else time.time_ns()
    body: dict[str, Any] = {
        "schema": SCHEMA,
        "layer_id": DataLayerId.EXEC_FEEDBACK.value,
        "session_id": data.session_id,
        "floor_id": data.floor_id,
        "tile_id": data.tile_id,
        "map_id": data.map_id,
        "context_generation": data.context_generation,
        "layer_version": data.layer_version,
        "timestamp_ns": ts,
        "command_id": data.command_id,
        "trace_id": data.trace_id,
        "phase": data.phase,
        "event": data.event,
        "accepted": data.accepted,
        "reason": data.reason,
        "safety_state": data.safety_state,
        "robot_pose": {
            "x": round(data.robot_x, 4),
            "y": round(data.robot_y, 4),
            "z": round(data.robot_z, 4),
            "yaw_deg": round(data.robot_yaw_deg, 2),
        },
    }
    if data.mode_id:
        body["mode_id"] = data.mode_id
    if data.mode_state:
        body["mode_state"] = data.mode_state
    if data.scene_type:
        body["scene_type"] = data.scene_type
    if data.events:
        body["events"] = data.events
    if data.world_fragments:
        body["world_fragments"] = data.world_fragments
    if data.extra:
        body["extra"] = data.extra
    return body
