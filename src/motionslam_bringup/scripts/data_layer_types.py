#!/usr/bin/env python3
"""端侧分层数据管理：层定义、视图状态、Catalog 快照（纯函数 + 内存目录）."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

CATALOG_SCHEMA = "motionslam.data_layer_catalog.v1"
ENVELOPE_SCHEMA = "motionslam.data_envelope.v1"


class DataLayerRole(str, Enum):
    """端侧两大任务角色."""

    PRODUCE = "produce"  # 任务 A：局部真实世界模型生产资料
    EXECUTE = "execute"  # 任务 B：边端指令安全执行与反馈


class ViewState(str, Enum):
    ABSENT = "absent"
    BUILDING = "building"
    READY = "ready"
    STALE = "stale"


class DataLayerId(str, Enum):
    SPATIAL_VOXEL = "spatial_voxel"
    TOPOLOGY_SUBGRAPH = "topology_subgraph"
    PERCEPTION_RGB = "perception_rgb"
    POSE_DIGEST = "pose_digest"
    LOCAL_PLANNING = "local_planning"
    EXEC_FEEDBACK = "exec_feedback"
    EXEC_DIRECTIVE = "exec_directive"


@dataclass(frozen=True)
class DataLayerSpec:
    layer_id: str
    role: str
    payload_schema: str
    transport: str  # tcp_json | tcp_binary | dds_reliable | dds_best_effort | local_only
    edge_port: int = 0
    default_hz: float = 0.0
    notes: str = ""


DEFAULT_LAYER_SPECS: tuple[DataLayerSpec, ...] = (
    DataLayerSpec(
        DataLayerId.SPATIAL_VOXEL.value,
        DataLayerRole.PRODUCE.value,
        "motionslam.voxel_diff.v1",
        "tcp_binary",
        edge_port=9876,
        default_hz=0.0,
        notes="HashVoxel/OctVox 增量；阶段 5 启用",
    ),
    DataLayerSpec(
        DataLayerId.TOPOLOGY_SUBGRAPH.value,
        DataLayerRole.PRODUCE.value,
        "motionslam.sparse_subgraph.v1",
        "tcp_json",
        edge_port=9877,
        default_hz=1.0,
        notes="关键帧拓扑 + frontier；SubgraphPublisher",
    ),
    DataLayerSpec(
        DataLayerId.PERCEPTION_RGB.value,
        DataLayerRole.PRODUCE.value,
        "motionslam.rgb_forward.v1",
        "tcp_binary",
        edge_port=9878,
        default_hz=15.0,
        notes="RGB 关键帧；与 subgraph_version 对齐",
    ),
    DataLayerSpec(
        DataLayerId.POSE_DIGEST.value,
        DataLayerRole.PRODUCE.value,
        "motionslam.pose_digest.v1",
        "dds_reliable",
        edge_port=0,
        default_hz=20.0,
        notes="位姿摘要；可选独立 DDS 话题",
    ),
    DataLayerSpec(
        DataLayerId.LOCAL_PLANNING.value,
        DataLayerRole.PRODUCE.value,
        "motionslam.local_planning_digest.v1",
        "local_only",
        edge_port=0,
        default_hz=10.0,
        notes="SCAN 局部规划摘要；端内消费，默认不上行",
    ),
    DataLayerSpec(
        DataLayerId.EXEC_DIRECTIVE.value,
        DataLayerRole.EXECUTE.value,
        "motionslam.thing_envelope.v1",
        "tcp_json",
        edge_port=9879,
        default_hz=0.0,
        notes="边端 → 端侧下行物模型信封（兼容 semantic_directive / action_group）",
    ),
    DataLayerSpec(
        DataLayerId.EXEC_FEEDBACK.value,
        DataLayerRole.EXECUTE.value,
        "motionslam.execution_feedback.v1",
        "tcp_json",
        edge_port=9880,
        default_hz=0.0,
        notes="端侧 → 边端执行态/安全态反馈",
    ),
)

# PGO applied 后需 mark stale 的生产层（几何/拓扑/感知派生）
STALE_ON_CONTEXT_GENERATION: frozenset[str] = frozenset(
    {
        DataLayerId.SPATIAL_VOXEL.value,
        DataLayerId.TOPOLOGY_SUBGRAPH.value,
        DataLayerId.PERCEPTION_RGB.value,
        DataLayerId.LOCAL_PLANNING.value,
    }
)


@dataclass
class LayerViewState:
    layer_id: str
    role: str
    payload_schema: str
    transport: str
    edge_port: int
    view_state: str = ViewState.ABSENT.value
    layer_version: int = 0
    context_generation_at_publish: int = 0
    last_publish_ns: int = 0
    last_error: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "layer_id": self.layer_id,
            "role": self.role,
            "payload_schema": self.payload_schema,
            "transport": self.transport,
            "edge_port": self.edge_port,
            "view_state": self.view_state,
            "layer_version": self.layer_version,
            "context_generation_at_publish": self.context_generation_at_publish,
            "last_publish_ns": self.last_publish_ns,
            "last_error": self.last_error,
        }


@dataclass
class LayerCatalog:
    session_id: str = ""
    floor_id: str = "floor_01"
    tile_id: str = "main"
    map_id: str = ""
    context_generation: int = 0
    catalog_version: int = 0
    layers: dict[str, LayerViewState] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.layers:
            self.layers = {
                spec.layer_id: LayerViewState(
                    layer_id=spec.layer_id,
                    role=spec.role,
                    payload_schema=spec.payload_schema,
                    transport=spec.transport,
                    edge_port=spec.edge_port,
                )
                for spec in DEFAULT_LAYER_SPECS
            }

    def bump_context_generation(self, generation: int) -> None:
        if generation <= self.context_generation:
            return
        self.context_generation = generation
        self.catalog_version += 1
        for layer_id in STALE_ON_CONTEXT_GENERATION:
            layer = self.layers.get(layer_id)
            if layer is None:
                continue
            if layer.view_state == ViewState.ABSENT.value:
                continue
            layer.view_state = ViewState.STALE.value
            layer.last_error = "context_generation_bump"

    def note_publish(
        self,
        layer_id: str,
        *,
        layer_version: int,
        context_generation: int,
        timestamp_ns: Optional[int] = None,
        error: str = "",
    ) -> None:
        layer = self.layers.get(layer_id)
        if layer is None:
            return
        ts = timestamp_ns if timestamp_ns is not None else time.time_ns()
        layer.layer_version = layer_version
        layer.context_generation_at_publish = context_generation
        layer.last_publish_ns = ts
        layer.last_error = error
        if error:
            layer.view_state = ViewState.STALE.value
        elif context_generation < self.context_generation:
            layer.view_state = ViewState.STALE.value
            layer.last_error = "published_behind_context_generation"
        else:
            layer.view_state = ViewState.READY.value
            layer.last_error = ""
        self.catalog_version += 1

    def note_building(self, layer_id: str) -> None:
        layer = self.layers.get(layer_id)
        if layer is None:
            return
        layer.view_state = ViewState.BUILDING.value
        self.catalog_version += 1

    def snapshot(self) -> dict[str, Any]:
        return build_catalog_snapshot(self)


def build_catalog_snapshot(catalog: LayerCatalog) -> dict[str, Any]:
    return {
        "schema": CATALOG_SCHEMA,
        "catalog_version": catalog.catalog_version,
        "timestamp_ns": time.time_ns(),
        "session_id": catalog.session_id,
        "floor_id": catalog.floor_id,
        "tile_id": catalog.tile_id,
        "map_id": catalog.map_id,
        "context_generation": catalog.context_generation,
        "layers": [catalog.layers[k].to_dict() for k in sorted(catalog.layers.keys())],
    }


def wrap_data_envelope(
    *,
    layer_id: str,
    payload_schema: str,
    payload: dict[str, Any],
    session_id: str,
    floor_id: str,
    tile_id: str,
    map_id: str,
    context_generation: int,
    layer_version: int,
    timestamp_ns: Optional[int] = None,
) -> dict[str, Any]:
    """可选统一上行信封；现有 payload 可原样发送，边端也可读 envelope."""
    ts = timestamp_ns if timestamp_ns is not None else time.time_ns()
    return {
        "schema": ENVELOPE_SCHEMA,
        "layer_id": layer_id,
        "payload_schema": payload_schema,
        "session_id": session_id,
        "floor_id": floor_id,
        "tile_id": tile_id,
        "map_id": map_id,
        "context_generation": context_generation,
        "layer_version": layer_version,
        "timestamp_ns": ts,
        "payload": payload,
    }


def layer_spec(layer_id: str) -> Optional[DataLayerSpec]:
    for spec in DEFAULT_LAYER_SPECS:
        if spec.layer_id == layer_id:
            return spec
    return None
