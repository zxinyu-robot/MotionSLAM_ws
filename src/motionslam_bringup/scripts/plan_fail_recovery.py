#!/usr/bin/env python3
"""规划失败恢复：漂移判定 + ICP/脱困决策（纯逻辑，便于单测）."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional


class RecoveryBranch(str, Enum):
    DRIFT_ICP = "drift_icp"
    ESCAPE = "escape"


@dataclass
class DriftThresholds:
    trans_m: float = 0.25
    yaw_deg: float = 8.0
    odom_jump_m: float = 0.45
    anchor_trans_m: float = 0.35
    anchor_yaw_deg: float = 10.0


@dataclass
class DriftAssessment:
    drifted: bool
    branch: RecoveryBranch
    reason: str
    trans_m: float = 0.0
    yaw_deg: float = 0.0
    source: str = "none"

    def to_payload(self) -> dict[str, Any]:
        return {
            "drifted": self.drifted,
            "branch": self.branch.value,
            "reason": self.reason,
            "trans_m": round(self.trans_m, 4),
            "yaw_deg": round(self.yaw_deg, 2),
            "source": self.source,
        }


def _normalize_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def yaw_delta(from_yaw: float, to_yaw: float) -> float:
    return abs(_normalize_yaw(to_yaw - from_yaw))


def parse_drift_estimate(raw: Optional[str]) -> tuple[float, float]:
    if not raw:
        return 0.0, 0.0
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return 0.0, 0.0
    return float(data.get("trans_m", 0.0)), float(data.get("yaw_deg", 0.0))


def assess_drift(
    *,
    drift_estimate_raw: Optional[str] = None,
    odom_jump_m: float = 0.0,
    anchor_trans_m: float = 0.0,
    anchor_yaw_deg: float = 0.0,
    relocation_fitness: Optional[float] = None,
    relocation_max_fitness: float = 0.35,
    thresholds: Optional[DriftThresholds] = None,
) -> DriftAssessment:
    """判定是否「飘了」：PGO 漂移 > 里程计跳变 > 锚点位姿偏差."""
    th = thresholds or DriftThresholds()

    pgo_trans, pgo_yaw = parse_drift_estimate(drift_estimate_raw)
    if pgo_trans >= th.trans_m or abs(pgo_yaw) >= th.yaw_deg:
        return DriftAssessment(
            drifted=True,
            branch=RecoveryBranch.DRIFT_ICP,
            reason="pgo_drift_estimate",
            trans_m=pgo_trans,
            yaw_deg=pgo_yaw,
            source="drift_estimate",
        )

    if odom_jump_m >= th.odom_jump_m:
        return DriftAssessment(
            drifted=True,
            branch=RecoveryBranch.DRIFT_ICP,
            reason="odom_jump",
            trans_m=odom_jump_m,
            source="odom_jump",
        )

    if anchor_trans_m >= th.anchor_trans_m or anchor_yaw_deg >= th.anchor_yaw_deg:
        return DriftAssessment(
            drifted=True,
            branch=RecoveryBranch.DRIFT_ICP,
            reason="anchor_pose_drift",
            trans_m=anchor_trans_m,
            yaw_deg=anchor_yaw_deg,
            source="dispatch_anchor",
        )

    if relocation_fitness is not None and relocation_fitness > relocation_max_fitness:
        return DriftAssessment(
            drifted=True,
            branch=RecoveryBranch.DRIFT_ICP,
            reason="relocation_fitness_high",
            trans_m=relocation_fitness,
            source="relocation_fitness",
        )

    return DriftAssessment(
        drifted=False,
        branch=RecoveryBranch.ESCAPE,
        reason="no_drift",
    )


def should_pause_replan(
    *,
    replan_fail_count: int,
    pause_after: int,
    emergency_bspline: bool = False,
) -> bool:
    if emergency_bspline:
        return True
    return replan_fail_count >= max(1, pause_after)
