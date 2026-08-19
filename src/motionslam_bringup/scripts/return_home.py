#!/usr/bin/env python3
"""端侧原路返航：面包屑记录 + 倒序 SCAN 参考路径（不依赖 PCT A* / 边端）。"""
from __future__ import annotations

import math
from typing import Optional, Sequence


def hypot(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def densify_xy(
    xy_list: Sequence[tuple[float, float]],
    *,
    step_m: float = 0.20,
) -> list[tuple[float, float]]:
    """把折线加密到约 step_m，避免 SCAN navi_mode=3 两点路径加速度超限。"""
    if len(xy_list) < 2 or step_m <= 0.0:
        return [(float(x), float(y)) for x, y in xy_list]
    out: list[tuple[float, float]] = [(float(xy_list[0][0]), float(xy_list[0][1]))]
    for i in range(1, len(xy_list)):
        ax, ay = out[-1]
        bx, by = float(xy_list[i][0]), float(xy_list[i][1])
        dist = hypot(ax, ay, bx, by)
        if dist < 1e-4:
            continue
        n = max(1, int(math.ceil(dist / step_m)))
        for k in range(1, n + 1):
            t = k / n
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return out


def record_breadcrumb(
    trail: list[tuple[float, float]],
    x: float,
    y: float,
    *,
    spacing_m: float = 0.60,
    max_poses: int = 400,
) -> bool:
    """位移超过 spacing 则追加；返回是否写入。"""
    if spacing_m <= 0.0:
        return False
    if not trail:
        trail.append((float(x), float(y)))
        return True
    lx, ly = trail[-1]
    if hypot(x, y, lx, ly) < spacing_m:
        return False
    trail.append((float(x), float(y)))
    overflow = len(trail) - max_poses
    if overflow > 0:
        del trail[:overflow]
    return True


def reverse_xy_for_scan(
    trail: Sequence[tuple[float, float]],
    current_xy: tuple[float, float],
    *,
    first_max_m: float = 0.55,
    min_end_dist_m: float = 0.80,
) -> list[tuple[float, float]]:
    """去程倒序给 SCAN navi_mode=3：首点距当前位姿 <= first_max_m，避免回填占用格。"""
    if len(trail) < 2:
        return []
    cx, cy = current_xy
    rev = list(reversed(trail))
    nearest_i = min(range(len(rev)), key=lambda i: hypot(rev[i][0], rev[i][1], cx, cy))
    rev = rev[nearest_i:]
    if len(rev) < 2:
        return []
    out: list[tuple[float, float]] = []
    hx, hy = rev[0]
    if hypot(hx, hy, cx, cy) > first_max_m:
        out.append((cx, cy))
        # 若当前点几乎等于最近面包屑，丢掉重复头
        if hypot(hx, hy, cx, cy) < 0.12:
            rev = rev[1:]
    out.extend((float(x), float(y)) for x, y in rev)
    if len(out) < 2:
        return []
    end = out[-1]
    if hypot(end[0], end[1], cx, cy) < min_end_dist_m:
        return []
    return out


def should_return_home(
    *,
    explicit: bool = False,
    glass_trap: bool = False,
    link_lost: bool = False,
    already_returning: bool = False,
    trail_len: int = 0,
) -> Optional[str]:
    """是否触发返航。already_returning 时不再重复。"""
    if already_returning or trail_len < 2:
        return None
    if explicit:
        return "explicit"
    if glass_trap:
        return "glass_trap"
    if link_lost:
        return "link_lost"
    return None


def uplink_is_lost(
    *,
    ever_ok: bool,
    last_ok_mono: float,
    now_mono: float,
    timeout_s: float,
    uplink_ok: Optional[bool],
) -> bool:
    """曾连通过，随后心跳丢失或超时。从未连通不算断网。"""
    if not ever_ok or timeout_s <= 0.0:
        return False
    if uplink_ok is False:
        return True
    if last_ok_mono <= 0.0:
        return False
    return (now_mono - last_ok_mono) >= timeout_s
