#!/usr/bin/env python3
"""为 Super-LIO 重定位生成降采样地图 (避免 relocation_node OOM).

默认读取 maps/PCD/scans_*.pcd 或 map.pcd, 输出 map_reloc.pcd.

用法 (容器内):
  python3 scripts/downsample_reloc_map.py
  python3 scripts/downsample_reloc_map.py --leaf 0.5 --input /ws/maps/map.pcd
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np


def read_pcd_xyz_i(path: str) -> np.ndarray:
    """读取 binary PCD (x y z intensity, float32)."""
    header: dict[str, str] = {}
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError(f"invalid PCD header: {path}")
            text = line.decode("ascii", errors="ignore").strip()
            if not text or text.startswith("#"):
                continue
            if text.startswith("DATA"):
                parts = text.split()
                header["DATA"] = parts[1] if len(parts) > 1 else "binary"
                break
            key, *rest = text.split(maxsplit=1)
            header[key] = rest[0] if rest else ""

    if header.get("DATA") != "binary":
        raise RuntimeError(f"only binary PCD supported: {path}")
    fields = header["FIELDS"].split()
    if fields[:4] != ["x", "y", "z", "intensity"]:
        raise RuntimeError(f"expected x y z intensity fields: {path}")

    n = int(header["POINTS"])
    with open(path, "rb") as f:
        while True:
            pos = f.tell()
            line = f.readline()
            if line.decode("ascii", errors="ignore").strip().startswith("DATA"):
                break
        data = np.fromfile(f, dtype=np.float32, count=n * 4)
    if data.size != n * 4:
        raise RuntimeError(f"truncated PCD: {path}")
    return data.reshape(-1, 4)


def voxel_mean(points: np.ndarray, leaf: float) -> np.ndarray:
    """向量化体素降采样 (每格取均值)."""
    if points.size == 0:
        return points.reshape(0, 4)
    vox = np.floor(points[:, :3] / leaf).astype(np.int64)
    vox -= vox.min(axis=0)
    stride = vox.max(axis=0) + 1
    keys = vox[:, 0] + stride[0] * (vox[:, 1] + stride[1] * vox[:, 2])
    order = np.argsort(keys)
    keys = keys[order]
    pts = points[order]
    breaks = np.concatenate(([0], np.where(np.diff(keys) != 0)[0] + 1, [len(keys)]))
    out = np.empty((len(breaks) - 1, 4), dtype=np.float32)
    for i in range(len(breaks) - 1):
        s, e = breaks[i], breaks[i + 1]
        out[i] = pts[s:e].mean(axis=0)
    return out


def filter_by_fragment_presence(
    parts: list[np.ndarray], leaf: float, min_fragments: int
) -> np.ndarray:
    """保留至少出现在 N 个独立 PCD 分片中的体素，抑制短暂动态点。"""
    if min_fragments <= 1 or len(parts) <= 1:
        return np.vstack(parts) if parts else np.empty((0, 4), dtype=np.float32)

    presence: dict[tuple[int, int, int], int] = {}
    for part in parts:
        voxels = np.floor(part[:, :3] / leaf).astype(np.int64)
        for voxel in set(map(tuple, voxels.tolist())):
            presence[voxel] = presence.get(voxel, 0) + 1

    kept: list[np.ndarray] = []
    for part in parts:
        voxels = np.floor(part[:, :3] / leaf).astype(np.int64)
        mask = np.fromiter(
            (presence[tuple(voxel)] >= min_fragments for voxel in voxels.tolist()),
            dtype=bool,
            count=len(voxels),
        )
        if np.any(mask):
            kept.append(part[mask])
    return np.vstack(kept) if kept else np.empty((0, 4), dtype=np.float32)


def filter_isolated_voxels(
    points: np.ndarray, leaf: float, min_neighbors: int
) -> np.ndarray:
    """去除没有足够相邻体素的孤立点，默认关闭。"""
    if min_neighbors <= 0 or points.size == 0:
        return points
    voxels = np.floor(points[:, :3] / leaf).astype(np.int64)
    occupied = set(map(tuple, voxels.tolist()))
    offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]
    keep = []
    for voxel in voxels.tolist():
        neighbor_count = sum(
            (voxel[0] + dx, voxel[1] + dy, voxel[2] + dz) in occupied
            for dx, dy, dz in offsets
        )
        keep.append(neighbor_count >= min_neighbors)
    return points[np.asarray(keep, dtype=bool)]


def resolve_inputs(input_path: str, maps_dir: str) -> list[str]:
    if input_path:
        return [input_path]
    pcd_dir = os.path.join(maps_dir, "PCD")
    scans = sorted(glob.glob(os.path.join(pcd_dir, "scans_*.pcd")))
    if scans:
        return scans
    default_map = os.path.join(maps_dir, "map.pcd")
    if os.path.isfile(default_map):
        return [default_map]
    raise FileNotFoundError(f"no map found under {maps_dir}")


def main() -> int:
    parser = argparse.ArgumentParser(description="生成重定位用降采样地图 map_reloc.pcd")
    parser.add_argument("--maps-dir", default=os.environ.get("MOTIONSLAM_MAPS", "/ws/maps"))
    parser.add_argument("--input", default="", help="单个 PCD; 默认 map.pcd 或 PCD/scans_*.pcd")
    parser.add_argument("--output", default="", help="默认 <maps-dir>/map_reloc.pcd")
    parser.add_argument("--leaf", type=float, default=0.5, help="体素边长 (m)")
    parser.add_argument(
        "--min-fragments",
        type=int,
        default=1,
        help="体素至少出现在多少个独立 scans 分片中；1=不做跨分片过滤",
    )
    parser.add_argument(
        "--min-neighbors",
        type=int,
        default=0,
        help="保留体素至少需要的 26 邻域数量；0=不做孤立点过滤",
    )
    args = parser.parse_args()

    inputs = resolve_inputs(args.input, args.maps_dir)
    output = args.output or os.path.join(args.maps_dir, "map_reloc.pcd")

    print(f"输入 {len(inputs)} 个 PCD, leaf={args.leaf} m")
    parts: list[np.ndarray] = []
    total_in = 0
    for i, path in enumerate(inputs, 1):
        pts = read_pcd_xyz_i(path)
        total_in += pts.shape[0]
        ds = voxel_mean(pts, args.leaf)
        parts.append(ds)
        print(f"  [{i}/{len(inputs)}] {os.path.basename(path)}: {pts.shape[0]:,} -> {ds.shape[0]:,}")

    merged = filter_by_fragment_presence(parts, args.leaf, args.min_fragments)
    if args.min_fragments > 1:
        print(
            f"跨分片过滤: min_fragments={args.min_fragments}, "
            f"保留候选 {merged.shape[0]:,} 点"
        )
    print(f"合并 {merged.shape[0]:,} 点, 最终体素化 ...")
    out = voxel_mean(merged, args.leaf)
    before_neighbors = out.shape[0]
    out = filter_isolated_voxels(out, args.leaf, args.min_neighbors)
    if args.min_neighbors > 0:
        print(
            f"邻域过滤: min_neighbors={args.min_neighbors}, "
            f"{before_neighbors:,} -> {out.shape[0]:,} 点"
        )

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {out.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {out.shape[0]}\n"
        "DATA binary\n"
    )
    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    with open(output, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(out.astype(np.float32).tobytes())

    out_mb = os.path.getsize(output) / (1024 * 1024)
    print(f"OK: {output}")
    print(f"  输入 {total_in:,} 点 -> 输出 {out.shape[0]:,} 点, {out_mb:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
