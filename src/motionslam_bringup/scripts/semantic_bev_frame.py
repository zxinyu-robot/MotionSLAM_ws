#!/usr/bin/env python3
"""BEV 语义栅格 → Token 上行帧（meta JSON + raw grid bytes）."""
from __future__ import annotations

import json
import struct
import time
from dataclasses import dataclass
from typing import Any, Optional, Tuple

MAGIC = b"MSTOK"
VERSION = 1
HEADER_STRUCT = struct.Struct(">5sBII")
META_SCHEMA = "motionslam.spatial_semantic_token.v1"
LAYER_ID = "spatial_voxel"


@dataclass
class SemanticTokenMetaInput:
    session_id: str
    floor_id: str
    tile_id: str
    map_id: str
    context_generation: int
    frame_id: str
    grid_width: int
    grid_height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_z: float
    pose_x: float
    pose_y: float
    pose_z: float
    pose_yaw_deg: float
    seq: int
    payload: bytes
    timestamp_ns: Optional[int] = None
    bev_source: str = "rgb_bev_stub"


def build_semantic_token_meta(data: SemanticTokenMetaInput) -> dict[str, Any]:
    ts = data.timestamp_ns if data.timestamp_ns is not None else time.time_ns()
    return {
        "schema": META_SCHEMA,
        "layer_id": LAYER_ID,
        "session_id": data.session_id,
        "floor_id": data.floor_id,
        "tile_id": data.tile_id,
        "map_id": data.map_id,
        "context_generation": data.context_generation,
        "seq": data.seq,
        "timestamp_ns": ts,
        "frame_id": data.frame_id,
        "bev_source": data.bev_source,
        "grid": {
            "width": data.grid_width,
            "height": data.grid_height,
            "resolution": data.resolution,
            "origin": {
                "x": round(data.origin_x, 4),
                "y": round(data.origin_y, 4),
                "z": round(data.origin_z, 4),
            },
        },
        "pose": {
            "x": round(data.pose_x, 4),
            "y": round(data.pose_y, 4),
            "z": round(data.pose_z, 4),
            "yaw_deg": round(data.pose_yaw_deg, 2),
        },
        "payload_len": len(data.payload),
    }


def pack_semantic_token_frame(meta: dict[str, Any], payload: bytes) -> bytes:
    meta_bytes = json.dumps(meta, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return HEADER_STRUCT.pack(MAGIC, VERSION, len(meta_bytes), len(payload)) + meta_bytes + payload


def unpack_semantic_token_frame(blob: bytes) -> Tuple[dict[str, Any], bytes, int]:
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
    json_off = min_len
    payload_off = json_off + json_len
    meta = json.loads(blob[json_off:payload_off].decode("utf-8"))
    payload = blob[payload_off:payload_off + payload_len]
    return meta, payload, total
