#!/usr/bin/env python3
"""MVPI1 ActionGroup[OBJNAV] 子集校验（纯函数）."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from task_context import TaskIdentityContext, validate_frontier_ids, validate_task_identity

SCHEMA = "motionslam.action_group.v1"
ALLOWED_TASK_TYPES = frozenset({"OBJNAV", "SEMANTIC_SEARCH"})
ALLOWED_STAGE_TYPES = frozenset({"NAVIGATE", "SEARCH", "REPORT"})
MVPI1_STAGE_IDS = frozenset(
    {
        "SEARCH_SEMANTIC_TARGET",
        "NAV_TO_DESTINATION",
        "REPORT_RESULT",
    }
)


@dataclass
class ActionGroupValidation:
    ok: bool
    reason: str = ""
    action_group: Optional[dict[str, Any]] = None


def _validate_waypoint(wp: dict[str, Any], index: int) -> Optional[str]:
    for key in ("x", "y", "z"):
        if key not in wp:
            return f"REJECT_WAYPOINT_{index}_MISSING_{key.upper()}"
        try:
            float(wp[key])
        except (TypeError, ValueError):
            return f"REJECT_WAYPOINT_{index}_BAD_{key.upper()}"
    return None


def _validate_stage(stage: dict[str, Any], index: int) -> Optional[str]:
    stage_id = str(stage.get("stage_id", "")).strip()
    stage_type = str(stage.get("stage_type", "")).strip().upper()
    if stage_type not in ALLOWED_STAGE_TYPES:
        return f"REJECT_STAGE_{index}_TYPE:{stage_type}"
    if stage_id and stage_id not in MVPI1_STAGE_IDS:
        return f"REJECT_STAGE_{index}_ID:{stage_id}"
    waypoints = stage.get("waypoints") or []
    if stage_type == "NAVIGATE" and not waypoints:
        return f"REJECT_STAGE_{index}_NO_WAYPOINTS"
    for wi, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            return f"REJECT_STAGE_{index}_WAYPOINT_{wi}_TYPE"
        err = _validate_waypoint(wp, wi)
        if err:
            return err
    return None


def validate_action_group(
    raw: dict[str, Any],
    ctx: TaskIdentityContext,
) -> ActionGroupValidation:
    schema = str(raw.get("schema", raw.get("schema_version", "")))
    if schema and schema not in (SCHEMA, "1.0.0"):
        return ActionGroupValidation(False, f"REJECT_SCHEMA:{schema}")

    command_id = raw.get("command_id")
    if not command_id:
        return ActionGroupValidation(False, "REJECT_MISSING_COMMAND_ID")

    task_type = str(raw.get("task_type", "OBJNAV")).upper()
    if task_type not in ALLOWED_TASK_TYPES:
        return ActionGroupValidation(False, f"REJECT_TASK_TYPE:{task_type}")

    identity_err = validate_task_identity(raw, ctx, require_lifecycle=True)
    if identity_err:
        return ActionGroupValidation(False, identity_err)

    stages = raw.get("stages") or []
    if not stages:
        return ActionGroupValidation(False, "REJECT_EMPTY_STAGES")
    for i, stage in enumerate(stages):
        if not isinstance(stage, dict):
            return ActionGroupValidation(False, f"REJECT_STAGE_{i}_TYPE")
        err = _validate_stage(stage, i)
        if err:
            return ActionGroupValidation(False, err)

    frontier_err = validate_frontier_ids(raw, ctx)
    if frontier_err:
        return ActionGroupValidation(False, frontier_err)

    return ActionGroupValidation(True, "ACCEPT", raw)


def extract_nav_waypoint(action_group: dict[str, Any]) -> Optional[dict[str, Any]]:
    """取第一个 NAVIGATE stage 的首个 waypoint（Phase 1 定向到达）."""
    for stage in action_group.get("stages") or []:
        if str(stage.get("stage_type", "")).upper() != "NAVIGATE":
            continue
        waypoints = stage.get("waypoints") or []
        if waypoints:
            return waypoints[0]
    return None


def extract_primary_stage_type(action_group: dict[str, Any]) -> Optional[str]:
    """返回第一个非 REPORT stage 的类型（SEARCH / NAVIGATE）."""
    for stage in action_group.get("stages") or []:
        stage_type = str(stage.get("stage_type", "")).upper()
        if stage_type == "REPORT":
            continue
        if stage_type in ALLOWED_STAGE_TYPES:
            return stage_type
    return None


def has_search_stage(action_group: dict[str, Any]) -> bool:
    for stage in action_group.get("stages") or []:
        if str(stage.get("stage_type", "")).upper() == "SEARCH":
            return True
    return False


def has_navigate_stage(action_group: dict[str, Any]) -> bool:
    for stage in action_group.get("stages") or []:
        if str(stage.get("stage_type", "")).upper() == "NAVIGATE":
            return True
    return False
