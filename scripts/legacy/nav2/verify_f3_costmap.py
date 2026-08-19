#!/usr/bin/env python3
"""F3 代价图语义: /lio/cloud_world 障碍点与 /grid_map/occupancy_inflate 空间一致.

用法 (ego_nav / slam_viz 运行中, 视野内应有障碍):
  python3 scripts/verify_f3_costmap.py --once

通过标准 (E2E F3):
  障碍高度带内 cloud 点与 inflate 点 XY 匹配率 recall/precision >= 阈值
  且两侧点数足够 (非空场 trivial pass)
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def read_xyz(msg: PointCloud2) -> list[tuple[float, float, float]]:
    pts = []
    for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        pts.append((float(x), float(y), float(z)))
    return pts


def subsample(pts: list[tuple[float, float, float]], max_n: int) -> list[tuple[float, float, float]]:
    if len(pts) <= max_n:
        return pts
    step = max(1, len(pts) // max_n)
    return pts[::step][:max_n]


def filter_z(
    pts: list[tuple[float, float, float]], z_min: float, z_max: float
) -> list[tuple[float, float, float]]:
    return [p for p in pts if z_min <= p[2] <= z_max]


def build_xy_grid(pts: list[tuple[float, float, float]], cell: float) -> set[tuple[int, int]]:
    grid: set[tuple[int, int]] = set()
    for x, y, _ in pts:
        grid.add((int(math.floor(x / cell)), int(math.floor(y / cell))))
    return grid


def match_ratio(
    src: list[tuple[float, float, float]],
    ref_grid: set[tuple[int, int]],
    cell: float,
    radius_cells: int,
) -> float:
    if not src:
        return 0.0
    hit = 0
    for x, y, _ in src:
        cx, cy = int(math.floor(x / cell)), int(math.floor(y / cell))
        ok = False
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if (cx + dx, cy + dy) in ref_grid:
                    ok = True
                    break
            if ok:
                break
        if ok:
            hit += 1
    return hit / len(src)


def sample_cloud(node: Node, topic: str, timeout: float) -> PointCloud2 | None:
    holder: list[PointCloud2] = []

    def cb(m: PointCloud2) -> None:
        holder.append(m)

    node.create_subscription(PointCloud2, topic, cb, 10)
    t0 = time.time()
    while not holder and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    return holder[0] if holder else None


def main() -> int:
    parser = argparse.ArgumentParser(description="F3 代价图语义验收")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--z-min", type=float, default=-0.20, help="障碍高度带下限 (world z)")
    parser.add_argument("--z-max", type=float, default=0.55, help="障碍高度带上限")
    parser.add_argument("--cell", type=float, default=0.05, help="匹配栅格 (m), 对齐 grid_map resolution")
    parser.add_argument("--match-radius", type=float, default=0.35, help="XY 匹配半径 (m)")
    parser.add_argument("--min-cloud", type=int, default=80, help="cloud 障碍点下限")
    parser.add_argument("--min-occ", type=int, default=30, help="inflate 点下限")
    parser.add_argument("--min-recall", type=float, default=0.45, help="cloud→occ 匹配率")
    parser.add_argument("--min-precision", type=float, default=0.35, help="occ→cloud 匹配率")
    parser.add_argument("--max-samples", type=int, default=2500)
    args = parser.parse_args()

    rclpy.init()
    node = Node("verify_f3_costmap")

    cloud_msg = sample_cloud(node, "/lio/cloud_world", 5.0)
    occ_msg = sample_cloud(node, "/grid_map/occupancy_inflate", 5.0)

    if not cloud_msg:
        print("ERROR: 无 /lio/cloud_world", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    if not occ_msg:
        print("ERROR: 无 /grid_map/occupancy_inflate (需 ego_nav 栈)", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    cloud_all = read_xyz(cloud_msg)
    occ_all = read_xyz(occ_msg)
    cloud_obs = subsample(filter_z(cloud_all, args.z_min, args.z_max), args.max_samples)
    occ_obs = subsample(occ_all, args.max_samples)

    print(f"F3: cloud 总点 {len(cloud_all)}, 障碍带 [{args.z_min},{args.z_max}] 点 {len(cloud_obs)}")
    print(f"    occupancy_inflate 点 {len(occ_obs)}")

    if len(cloud_obs) < args.min_cloud or len(occ_obs) < args.min_occ:
        print(
            f"SKIP: 障碍点过少 (cloud>={args.min_cloud}, occ>={args.min_occ}); "
            "请在有障碍场景或建图/导航栈运行中复测",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 2

    radius_cells = max(1, int(math.ceil(args.match_radius / args.cell)))
    occ_grid = build_xy_grid(occ_obs, args.cell)
    cloud_grid = build_xy_grid(cloud_obs, args.cell)

    recall = match_ratio(cloud_obs, occ_grid, args.cell, radius_cells)
    precision = match_ratio(occ_obs, cloud_grid, args.cell, radius_cells)

    print(f"  recall (cloud→occ): {recall:.2f} (>= {args.min_recall})")
    print(f"  precision (occ→cloud): {precision:.2f} (>= {args.min_precision})")

    node.destroy_node()
    rclpy.shutdown()

    if recall >= args.min_recall and precision >= args.min_precision:
        print("OK: F3 代价图语义通过")
        return 0
    print("FAIL: F3 匹配率不足", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
