#!/usr/bin/env python3
"""MVPI1 任务身份与端侧健康门控上下文（Directive / ActionGroup 共用）."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Optional

ALLOWED_FRAMES = frozenset({"world", "map"})


@dataclass
class TaskIdentityContext:
    """端侧当前会话与健康状态，用于下行任务校验."""

    local_session_id: str = "demo_live"
    local_floor_id: str = "floor_01"
    local_map_id: str = "demo_live_map"
    local_frame_id: str = "world"
    local_map_version: int = 0
    context_generation: int = 0
    lifecycle_active: bool = False
    localization_ok: bool = True
    pgo_state: str = "IDLE"
    odom_age_s: float = 0.0
    max_odom_age_s: float = 2.0
    known_frontier_ids: set[str] = field(default_factory=set)
    require_known_frontier: bool = False
    local_robot_id: str = "go2_001"
    local_platform: str = "unitree_go2_edu"
    local_capability_tags: set[str] = field(
        default_factory=lambda: {"Navigate", "ReturnHome", "RecoverGlass", "ExploreFrontier"}
    )
    now_ns: Optional[int] = None

    def now(self) -> int:
        return self.now_ns if self.now_ns is not None else time.time_ns()


def _norm(value: Any) -> str:
    return str(value).strip()


def validate_task_identity(
    raw: dict[str, Any],
    ctx: TaskIdentityContext,
    *,
    require_lifecycle: bool = True,
    check_context_generation: bool = True,
) -> Optional[str]:
    """公共身份/健康校验。返回 REJECT_* 原因码，通过则 None."""
    session = raw.get("session_id")
    if session is not None and _norm(session) != _norm(ctx.local_session_id):
        return "REJECT_SESSION"

    floor = raw.get("floor_id")
    if floor is not None and _norm(floor) != _norm(ctx.local_floor_id):
        return "REJECT_FLOOR"

    map_id = raw.get("map_id")
    if map_id is not None and _norm(map_id) != _norm(ctx.local_map_id):
        return "REJECT_MAP"

    frame = raw.get("frame_id")
    if frame is not None:
        frame_s = _norm(frame)
        if frame_s not in ALLOWED_FRAMES:
            return f"REJECT_FRAME:{frame_s}"
        if frame_s != _norm(ctx.local_frame_id):
            return "REJECT_FRAME"

    map_version = raw.get("map_version", raw.get("version"))
    if map_version is not None:
        try:
            mv = int(map_version)
        except (TypeError, ValueError):
            return "REJECT_VERSION_PARSE"
        if mv > ctx.local_map_version:
            return "REJECT_VERSION"

    if check_context_generation:
        gen = raw.get("context_generation")
        if gen is not None:
            try:
                cg = int(gen)
            except (TypeError, ValueError):
                return "REJECT_CONTEXT_PARSE"
            if cg > ctx.context_generation:
                return "REJECT_CONTEXT"

    created = raw.get("created_ns", raw.get("issued_at_ns"))
    ttl_ms = raw.get("ttl_ms", 10000)
    try:
        ttl_ms_i = int(ttl_ms)
    except (TypeError, ValueError):
        ttl_ms_i = 10000
    if created is not None:
        try:
            created_ns = int(created)
        except (TypeError, ValueError):
            return "REJECT_TIME_PARSE"
        age_ms = (ctx.now() - created_ns) / 1_000_000.0
        if age_ms > ttl_ms_i:
            return "REJECT_EXPIRED"

    if require_lifecycle and not ctx.lifecycle_active:
        return "REJECT_LIFECYCLE"

    if not ctx.localization_ok or ctx.odom_age_s > ctx.max_odom_age_s:
        return "REJECT_LOCALIZATION"

    if ctx.pgo_state.upper() == "OPTIMIZING":
        return "REJECT_PGO_OPTIMIZING"

    return None


def validate_frontier_ids(
    raw: dict[str, Any],
    ctx: TaskIdentityContext,
) -> Optional[str]:
    frontier_ids = raw.get("candidate_frontier_ids") or []
    if ctx.require_known_frontier and frontier_ids:
        unknown = [fid for fid in frontier_ids if _norm(fid) not in ctx.known_frontier_ids]
        if unknown and len(unknown) == len(frontier_ids):
            return "REJECT_UNKNOWN_FRONTIER"
    return None
