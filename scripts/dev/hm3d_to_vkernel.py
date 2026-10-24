#!/usr/bin/env python3
"""hm3d_to_vkernel.py — HM3D glb 场景 → vkernel_sim 可消费的资产。

输出 npz:
  points:  (N,4) float32 XYZI —— 场景表面采样点云（world 系，供 /lio/cloud_world）
  hm_origin: (2,)  高度图原点 (x0,y0)
  hm_res:    高度图分辨率
  hm_levels: (H,W,K) float32 —— 每格候选地面层 z（NaN 填充），已加 body 高度偏移

地面层检测：每 (x,y) 格对点 z 做直方图，水平面（地板/台面）表现为尖峰，
墙面是均匀分布。取超过密度阈值的 z 峰值作为候选层。
机器人 z = 距当前 z 最近的候选层（容差内），实现多层/楼梯自然过渡。

用法:
  hm3d_to_vkernel.py SCENE.glb -o out.npz [--voxel 0.05] [--cell 0.2] [--body-z 0.35]
"""
import argparse, numpy as np, trimesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("glb")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--cell", type=float, default=0.2)
    ap.add_argument("--body-z", type=float, default=0.35)
    ap.add_argument("--sample", type=int, default=3_000_000)
    ap.add_argument("--zbin", type=float, default=0.05)
    ap.add_argument("--peak-min", type=int, default=30, help="峰值最少点数")
    ap.add_argument("--up-nz", type=float, default=0.7, help="可立足面法向 z 下限")
    a = ap.parse_args()

    scene = trimesh.load(a.glb)
    concat = scene.to_geometry()
    pts, fidx = trimesh.sample.sample_surface(concat, a.sample)
    pts = pts.astype(np.float32)
    # 可立足面 = 朝上表面（法向 z 分量大）；楼板底面/墙面不能当地面层，
    # 否则机器人会"站"进楼板底（下层天花板）
    up = concat.face_normals[fidx][:, 2] > a.up_nz

    # 体素抽稀
    ground = pts[up]
    key = np.floor(pts / a.voxel).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    pts = pts[np.sort(idx)]
    print(f"sampled {a.sample} -> voxel {len(pts)}")
    xyzi = np.hstack([pts, np.full((len(pts), 1), 40.0, np.float32)])

    # 高度图：每格 z 直方图找峰
    x0, y0 = pts[:, :2].min(0)
    x1, y1 = pts[:, :2].max(0)
    res = a.cell
    W = int(np.ceil((x1 - x0) / res)) + 1
    H = int(np.ceil((y1 - y0) / res)) + 1
    zmin, zmax = pts[:, 2].min(), pts[:, 2].max()
    nbins = int(np.ceil((zmax - zmin) / a.zbin)) + 1
    levels = np.full((H, W, 8), np.nan, np.float32)

    ix = np.clip(((ground[:, 0] - x0) / res).astype(int), 0, W - 1)
    iy = np.clip(((ground[:, 1] - y0) / res).astype(int), 0, H - 1)
    ib = np.clip(((ground[:, 2] - zmin) / a.zbin).astype(int), 0, nbins - 1)

    # 每格直方图（编码 key = cell_id * nbins + zbin）
    cell_id = iy * W + ix
    enc = cell_id.astype(np.int64) * nbins + ib
    keys, counts = np.unique(enc, return_counts=True)
    for k, c in zip(keys, counts):
        if c < a.peak_min:
            continue
        cid, zb = divmod(int(k), nbins)
        cy, cx = divmod(cid, W)
        lv = levels[cy, cx]
        slot = np.argmax(np.isnan(lv))
        if slot < lv.shape[0]:
            lv[slot] = zmin + zb * a.zbin + a.body_z

    np.savez_compressed(a.out, points=xyzi, hm_origin=np.array([x0, y0]),
                        hm_res=np.float32(res), hm_levels=levels)
    n_cells = np.sum(~np.isnan(levels[:, :, 0]))
    print(f"heightmap {H}x{W}, {n_cells} cells w/ levels; saved {a.out}")


if __name__ == "__main__":
    main()
