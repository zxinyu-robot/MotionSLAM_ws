#!/usr/bin/env python3
"""Go2FrontVideoData DDS 字段原样转发帧（meta JSON + raw bytes）."""
from __future__ import annotations

import json
import struct
import time
from dataclasses import dataclass
from typing import Any, Optional, Tuple

MAGIC = b"MSRGB"
VERSION = 1
HEADER_STRUCT = struct.Struct(">5sBII")  # magic(5), ver, json_len, payload_len
META_SCHEMA = "motionslam.rgb_forward.v1"


LAYER_ID = "perception_rgb"


@dataclass
class RgbForwardMetaInput:
    session_id: str
    stream_field: str
    payload: bytes
    time_frame: int
    seq: int
    floor_id: str = "floor_01"
    tile_id: str = "main"
    map_id: str = ""
    subgraph_version: int = 0
    context_generation: int = 0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_z: float = 0.0
    pose_yaw_deg: float = 0.0
    timestamp_ns: Optional[int] = None
    topic: str = "/frontvideostream"
    msg_type: str = "unitree_go/msg/Go2FrontVideoData"


def build_rgb_forward_meta(data: RgbForwardMetaInput) -> dict[str, Any]:
    ts = data.timestamp_ns if data.timestamp_ns is not None else time.time_ns()
    return {
        "schema": META_SCHEMA,
        "layer_id": LAYER_ID,
        "session_id": data.session_id,
        "floor_id": data.floor_id,
        "tile_id": data.tile_id,
        "map_id": data.map_id,
        "topic": data.topic,
        "msg_type": data.msg_type,
        "stream_field": data.stream_field,
        "time_frame": data.time_frame,
        "seq": data.seq,
        "timestamp_ns": ts,
        "subgraph_version": data.subgraph_version,
        "context_generation": data.context_generation,
        "payload_len": len(data.payload),
        "pose": {
            "x": round(data.pose_x, 4),
            "y": round(data.pose_y, 4),
            "z": round(data.pose_z, 4),
            "yaw_deg": round(data.pose_yaw_deg, 2),
        },
    }


def pack_rgb_forward_frame(meta: dict[str, Any], payload: bytes) -> bytes:
    meta_bytes = json.dumps(meta, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return HEADER_STRUCT.pack(MAGIC, VERSION, len(meta_bytes), len(payload)) + meta_bytes + payload


def unpack_rgb_forward_frame(blob: bytes) -> Tuple[dict[str, Any], bytes, int]:
    """返回 (meta, payload, consumed_bytes)."""
    min_len = HEADER_STRUCT.size
    if len(blob) < min_len:
        raise ValueError("frame too short")
    magic, ver, json_len, payload_len = HEADER_STRUCT.unpack_from(blob, 0)
    if magic != MAGIC:
        raise ValueError(f"bad magic: {magic!r}")
    if ver != VERSION:
        raise ValueError(f"unsupported version: {ver}")
    total = min_len + json_len + payload_len
    if len(blob) < total:
        raise ValueError("incomplete frame")
    off = min_len
    meta = json.loads(blob[off : off + json_len].decode("utf-8"))
    off += json_len
    payload = blob[off : off + payload_len]
    return meta, payload, total
