#!/usr/bin/env python3
"""MVPI1 语义任务：PoseStamped / ActionGroup → BT Subgoal（纯函数）."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from geometry_msgs.msg import PoseStamped


@dataclass
class SemanticSubgoal:
    x: float
    y: float
    z: float
    frame_id: str = "world"
    command_id: str = ""
    language_query: str = ""


def subgoal_from_pose(msg: PoseStamped, body_height: float = 0.35) -> SemanticSubgoal:
    z = float(msg.pose.position.z)
    if abs(z) < 1e-3:
        z = body_height
    return SemanticSubgoal(
        x=float(msg.pose.position.x),
        y=float(msg.pose.position.y),
        z=z,
        frame_id=str(msg.header.frame_id or "world"),
    )


def subgoal_from_waypoint(wp: dict[str, Any], frame_id: str = "world") -> SemanticSubgoal:
    return SemanticSubgoal(
        x=float(wp["x"]),
        y=float(wp["y"]),
        z=float(wp.get("z", 0.35)),
        frame_id=str(wp.get("frame_id", frame_id)),
    )


def quat_to_yaw(qz: float, qw: float) -> float:
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def heading_to_goal(
    from_xy: tuple[float, float],
    to_xy: tuple[float, float],
) -> tuple[float, float]:
    """Return (qz, qw) for planar heading."""
    heading = math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])
    return math.sin(heading * 0.5), math.cos(heading * 0.5)
