#!/usr/bin/env python3
"""Task Intent（目标架构边端入口）纯逻辑：校验 + 反馈构造，无 ROS 依赖可单测。

schema: motionslam.task_intent.v1
{
  "schema": "motionslam.task_intent.v1",
  "task_id": "task-...",
  "robot":   {"platform": "unitree_go2_edu", "sn": "go2_001"},
  "world_ref": {"map_id": "demo_live_map", "floor_id": "floor_01",
                "frame_id": "world", "revision": 1},
  "skills":  [{"name": "Navigate", "goal": {"x": 2.0, "y": 0.5, "z": 0.35}},
              {"name": "ReturnHome"}],
  "policy":  {"deadline_s": 120.0}
}
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

from behavior_mode import (
    ALLOWED_PLATFORMS,
    ALLOWED_SKILLS,
    _has_forbidden_control,
    _norm,
)

SCHEMA_INTENT = "motionslam.task_intent.v1"
SCHEMA_FEEDBACK = "motionslam.execution_feedback.v1"


@dataclass
class IntentGoal:
    x: float
    y: float
    z: float
    frame_id: str = "world"


@dataclass
class IntentSkill:
    name: str
    goal: Optional[IntentGoal] = None


@dataclass
class TaskIntent:
    task_id: str = ""
    platform: str = ""
    robot_id: str = ""
    map_id: str = ""
    floor_id: str = ""
    frame_id: str = "world"
    revision: int = -1
    skills: list[IntentSkill] = field(default_factory=list)
    deadline_s: float = 0.0


def _err(code: str) -> str:
    return code


def validate_task_intent(
    raw: Any,
    *,
    allowed_skills: Optional[set[str]] = None,
    local_platform: str = "",
    local_robot_id: str = "",
) -> tuple[Optional[TaskIntent], Optional[str]]:
    """校验 Task Intent；返回 (intent, None) 或 (None, REJECT_*)。"""
    if not isinstance(raw, dict):
        return None, _err("REJECT_INTENT_TYPE")
    forbidden = _has_forbidden_control(raw)
    if forbidden:
        return None, _err(forbidden)
    schema = _norm(raw.get("schema", ""))
    if schema != SCHEMA_INTENT:
        return None, _err(f"REJECT_SCHEMA:{schema or 'missing'}")

    task_id = _norm(raw.get("task_id", ""))
    if not task_id:
        return None, _err("REJECT_MISSING_TASK_ID")

    robot = raw.get("robot") if isinstance(raw.get("robot"), dict) else {}
    platform = _norm(robot.get("platform", ""))
    if not platform:
        return None, _err("REJECT_MISSING_ROBOT")
    if platform not in ALLOWED_PLATFORMS:
        return None, _err(f"REJECT_ROBOT_PLATFORM:{platform}")
    sn = _norm(robot.get("sn") or robot.get("robot_id") or "")
    if not sn:
        return None, _err("REJECT_MISSING_ROBOT_ID")
    if local_platform and platform != _norm(local_platform):
        return None, _err("REJECT_ROBOT_MISMATCH")
    if local_robot_id and sn != _norm(local_robot_id):
        return None, _err("REJECT_ROBOT_MISMATCH")

    world = raw.get("world_ref")
    if not isinstance(world, dict):
        return None, _err("REJECT_MISSING_WORLD_REF")
    map_id = _norm(world.get("map_id", ""))
    if not map_id:
        return None, _err("REJECT_MISSING_MAP_ID")
    try:
        revision = int(world.get("revision", -1))
    except (TypeError, ValueError):
        return None, _err("REJECT_BAD_REVISION")
    if revision < 0:
        return None, _err("REJECT_BAD_REVISION")

    skills_raw = raw.get("skills")
    if not isinstance(skills_raw, list) or not skills_raw:
        return None, _err("REJECT_MISSING_SKILLS")
    cap = allowed_skills if allowed_skills is not None else set(ALLOWED_SKILLS)
    skills: list[IntentSkill] = []
    for i, item in enumerate(skills_raw):
        if not isinstance(item, dict):
            return None, _err(f"REJECT_SKILL_{i}_TYPE")
        name = _norm(item.get("name") or item.get("skill") or "")
        if not name:
            return None, _err(f"REJECT_SKILL_{i}_MISSING_NAME")
        if name not in ALLOWED_SKILLS or name not in cap:
            return None, _err(f"REJECT_SKILL_UNSUPPORTED:{name}")
        goal: Optional[IntentGoal] = None
        if name == "Navigate":
            g = item.get("goal")
            if not isinstance(g, dict):
                return None, _err(f"REJECT_SKILL_{i}_MISSING_GOAL")
            try:
                gx, gy, gz = float(g["x"]), float(g["y"]), float(g["z"])
            except (KeyError, TypeError, ValueError):
                return None, _err(f"REJECT_SKILL_{i}_BAD_GOAL")
            goal = IntentGoal(gx, gy, gz, _norm(g.get("frame_id") or world.get("frame_id") or "world"))
        skills.append(IntentSkill(name=name, goal=goal))

    policy = raw.get("policy") if isinstance(raw.get("policy"), dict) else {}
    deadline = 0.0
    if policy.get("deadline_s") is not None:
        try:
            deadline = float(policy["deadline_s"])
        except (TypeError, ValueError):
            return None, _err("REJECT_BAD_DEADLINE")
        if deadline <= 0:
            return None, _err("REJECT_BAD_DEADLINE")

    return (
        TaskIntent(
            task_id=task_id,
            platform=platform,
            robot_id=sn,
            map_id=map_id,
            floor_id=_norm(world.get("floor_id", "")),
            frame_id=_norm(world.get("frame_id") or "world"),
            revision=revision,
            skills=skills,
            deadline_s=deadline,
        ),
        None,
    )


def build_feedback(
    *,
    task_id: str,
    state: str,
    event: str,
    progress: Optional[dict[str, Any]] = None,
    detail: str = "",
    timestamp_ns: int = 0,
) -> dict[str, Any]:
    """Execution Feedback（目标架构上行）。"""
    return {
        "schema": SCHEMA_FEEDBACK,
        "task_id": task_id,
        "state": state,
        "event": event,
        "progress": progress or {},
        "detail": detail,
        "timestamp_ns": timestamp_ns,
    }
