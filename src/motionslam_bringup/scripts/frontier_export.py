#!/usr/bin/env python3
"""从关键帧拓扑导出探索 frontier（P2-3，scan-planner 表接入前的几何近似）."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from subgraph_snapshot import FrontierView, KeyframeView


@dataclass
class FrontierExportConfig:
    min_dist_m: float = 1.0
    max_dist_m: float = 12.0
    max_frontiers: int = 8
    ray_count: int = 8
    ray_dist_m: float = 3.0
    min_ray_frontiers: int = 4


def _dist_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def _geom_score(dist_m: float, max_dist_m: float) -> float:
    if max_dist_m <= 1e-6:
        return 1.0
    return max(0.05, min(1.0, dist_m / max_dist_m))


def export_frontiers_from_keyframes(
    robot_x: float,
    robot_y: float,
    robot_z: float,
    robot_yaw: float,
    keyframes: list[KeyframeView],
    *,
    config: Optional[FrontierExportConfig] = None,
) -> list[FrontierView]:
    """从关键帧链 + 射线采样生成 frontier 候选（单调 frontier_id）."""
    cfg = config or FrontierExportConfig()
    out: list[FrontierView] = []
    seen: set[tuple[float, float]] = set()

    for kf in keyframes:
        dist = _dist_xy(robot_x, robot_y, kf.x, kf.y)
        if dist < cfg.min_dist_m or dist > cfg.max_dist_m:
            continue
        key = (round(kf.x, 2), round(kf.y, 2))
        if key in seen:
            continue
        seen.add(key)
        z = kf.z if abs(kf.z) > 1e-3 else robot_z
        out.append(
            FrontierView(
                frontier_id=f"kf_{kf.keyframe_id}",
                x=kf.x,
                y=kf.y,
                z=z,
                geom_score=_geom_score(dist, cfg.max_dist_m),
            )
        )

    out.sort(key=lambda f: f.geom_score, reverse=True)
    out = out[: cfg.max_frontiers]

    if len(out) < cfg.min_ray_frontiers:
        step = 2.0 * math.pi / max(cfg.ray_count, 1)
        for i in range(cfg.ray_count):
            bearing = robot_yaw + i * step
            x = robot_x + cfg.ray_dist_m * math.cos(bearing)
            y = robot_y + cfg.ray_dist_m * math.sin(bearing)
            key = (round(x, 2), round(y, 2))
            if key in seen:
                continue
            seen.add(key)
            out.append(
                FrontierView(
                    frontier_id=f"ray_{i:02d}",
                    x=x,
                    y=y,
                    z=robot_z,
                    geom_score=_geom_score(cfg.ray_dist_m, cfg.max_dist_m),
                )
            )

    return out[: cfg.max_frontiers]
