#!/usr/bin/env python3
"""云端闭环 3D 点云视频渲染（Foxglove 风格）：订阅 /lio/cloud_world +
/lio/robo/odom + /demo/mission/event，mission_succeeded/failed 或超时后
用 matplotlib 3D 渲染 mp4——暗色底、按高度 jet 着色的点云、地面网格、
机器人朝向箭头与轨迹、HUD。

用法（容器内）:
    python3 cloud_video_render_3d.py \
        --out /ws/cross_floor_3d.mp4 --timeout 480
"""
import argparse
import json
import math
import struct
import sys

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
except ImportError:  # --load-npz 离线渲染不需要 ROS
    rclpy = None
    Node = object


def _decode_xyz(msg):
    """从 PointCloud2 取 xyz 列（x,y,z float32 固定 12 字节偏移）。"""
    offs = {f.name: f.offset for f in msg.fields}
    step = msg.point_step
    data = msg.data
    n = msg.width * msg.height
    arr = np.frombuffer(bytes(data[: n * step]), dtype=np.uint8).reshape(n, step)
    x = arr[:, offs["x"] : offs["x"] + 4].copy().view(np.float32)
    y = arr[:, offs["y"] : offs["y"] + 4].copy().view(np.float32)
    z = arr[:, offs["z"] : offs["z"] + 4].copy().view(np.float32)
    return np.stack([x, y, z], axis=1)


class Recorder(Node):
    def __init__(self, cloud_topic, odom_topic, occ_topic):
        super().__init__("cloud_video_render_3d")
        self.traj = []  # (t, x, y, z, yaw)
        self.cloud = None  # 最新一帧点云 Nx3
        self.occ_rows = []  # [(t, Nx3)] SCAN 体素占据增量
        self.events = []
        self.done = None
        self._t0 = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos)
        occ_qos = QoSProfile(depth=50)
        occ_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        occ_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, occ_topic, self._on_occ, occ_qos)
        self.create_subscription(String, "/demo/mission/event", self._on_event, 10)

    def _on_odom(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        # 记录姿态 + 当时的点云快照（浅拷贝引用；点云本就全量重发）
        self.traj.append((now - self._t0, p.x, p.y, p.z, yaw))

    def _on_cloud(self, msg):
        try:
            self.cloud = _decode_xyz(msg)
        except Exception as e:
            self.get_logger().warn(f"cloud decode failed: {e}")

    def _on_occ(self, msg):
        if self._t0 is None:
            return
        try:
            pts = _decode_xyz(msg)
            if len(pts):
                now = self.get_clock().now().nanoseconds * 1e-9
                self.occ_rows.append((now - self._t0, pts))
        except Exception:
            pass

    def _on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except Exception:
            return
        name = ev.get("event", "")
        self.events.append((name, ev.get("subgoal_index")))
        if name == "mission_subgoal_reached":
            self.get_logger().info(f"[event] subgoal {ev.get('subgoal_index')}")
        if name in ("mission_succeeded", "mission_failed", "mission_rejected"):
            self.done = name
            self.get_logger().info(f"[event] {name}")


def _footprint_cloud(cloud, x, y, z, yaw, radius=3.0):
    """只保留机器人附近的点（俯视图局部视野），返回 Nx3。"""
    if cloud is None:
        return None
    dx = cloud[:, 0] - x
    dy = cloud[:, 1] - y
    m = (dx * dx + dy * dy) <= radius * radius
    return cloud[m]


def _load_map_params(yaml_path):
    import yaml

    data = yaml.safe_load(open(yaml_path))
    return data["vkernel_sim"]["ros__parameters"]


def _height_at(x, y, ramps, floors):
    for x1, y1, x2, y2, z1, z2, axis in ramps:
        if x1 <= x <= x2 and y1 <= y <= y2:
            t = (x - x1) / max(1e-6, x2 - x1) if axis < 0.5 else (y - y1) / max(1e-6, y2 - y1)
            return z1 + (z2 - z1) * min(1.0, max(0.0, t))
    for x1, y1, x2, y2, z in floors:
        if x1 <= x <= x2 and y1 <= y <= y2:
            return z
    return 0.0


def _surface_cloud(params, step=0.06):
    """把地面/坡道/楼层平台按高度场采样成密集点云面（仅渲染用，
    不进 /lio/cloud_world——地面点会被 SCAN 当障碍）。"""

    def chunks(flat, n):
        return [tuple(flat[i : i + n]) for i in range(0, len(flat) - n + 1, n)]

    floors = chunks(params.get("floors", []), 5)
    ramps = chunks(params.get("ramps", []), 7)
    walls = chunks(params.get("walls", []), 5)
    geo = list(floors) + list(ramps) + list(walls)
    if not geo:
        return None
    xs = [g[0] for g in geo] + [g[2] for g in geo]
    ys = [g[1] for g in geo] + [g[3] for g in geo]
    x0, x1 = min(xs) - 0.4, max(xs) + 0.4
    y0, y1 = min(ys) - 0.4, max(ys) + 0.4
    gx = np.arange(x0, x1, step)
    gy = np.arange(y0, y1, step)
    xx, yy = np.meshgrid(gx, gy)
    zz = np.vectorize(lambda a, b: _height_at(a, b, ramps, floors))(xx, yy)
    rng = np.random.default_rng(7)
    zz = zz + rng.uniform(-0.015, 0.015, zz.shape)  # 真实雷达厚度噪声
    pts = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)
    # 稀疏散点（真机点云里的浮尘/孤立点）
    nd = 350
    dust = np.stack(
        [
            rng.uniform(x0, x1, nd),
            rng.uniform(y0, y1, nd),
            rng.uniform(0.0, float(zz.max()) + 0.4, nd),
        ],
        axis=1,
    ).astype(np.float32)
    return np.concatenate([pts, dust], axis=0)


def _accumulate_occ(occ_rows, upto_t):
    """体素占据增量累积到时刻 t（按 0.05m 体素键去重，保留最新 z）。"""
    vox = {}
    for t, pts in occ_rows:
        if t > upto_t:
            break
        for p in pts:
            key = (round(float(p[0]) * 20), round(float(p[1]) * 20), round(float(p[2]) * 20))
            vox[key] = (float(p[0]), float(p[1]), float(p[2]))
    if not vox:
        return None
    return np.array(list(vox.values()), dtype=np.float32)


def _occ_prefix(occ_rows, idxs_t, vox=0.05):
    """预计算每个帧时刻的占据快照。

    体素点量可达千万级：不用 python dict，栅格化成 first_seen
    时间数组（一格=一个体素，存最早出现时刻），每帧取 <=t 的格子。"""
    if not occ_rows:
        return [None] * len(idxs_t)
    all_pts = np.concatenate([p.reshape(-1, 3) for _, p in occ_rows if len(p)])
    mins = all_pts.min(axis=0).ravel()
    dims = ((all_pts.max(axis=0) - mins) / vox).astype(np.int64) + 2
    W, H, D = int(dims[0]), int(dims[1]), int(dims[2])
    first_t = np.full(W * H * D, np.inf, dtype=np.float64)
    for t, p in sorted(occ_rows, key=lambda r: r[0]):
        if not len(p):
            continue
        idx = ((p.reshape(-1, 3) - mins) / vox).astype(np.int64)
        flat = (idx[:, 0] * H + idx[:, 1]) * D + idx[:, 2]
        first_t[flat] = np.minimum(first_t[flat], t)
    snaps = []
    for t in idxs_t:
        sel = np.nonzero(first_t <= t)[0]
        if not len(sel):
            snaps.append(None)
            continue
        ix = sel // (H * D)
        iy = (sel // D) % H
        iz = sel % D
        pts = np.stack([ix, iy, iz], axis=1) * vox + mins
        snaps.append(pts.astype(np.float32))
    return snaps


def render_dual(traj, cloud, occ_rows, surface, wpts, out_path, fps=15):
    """双视口：左=实时点云，右=在线体素占据 + 轨迹带 + 航点球。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    n = len(traj)
    step = max(1, math.ceil(n / (fps * 75)))
    idxs = list(range(0, n, step))
    occ_snaps = _occ_prefix(occ_rows, [traj[i][0] for i in idxs], vox=0.08)

    xs = [p[1] for p in traj]
    ys = [p[2] for p in traj]
    zs = [p[3] for p in traj]
    x0, x1 = min(xs) - 0.6, max(xs) + 0.6
    y0, y1 = min(ys) - 0.6, max(ys) + 0.6
    zmax = max(max(zs), 1.3)
    if cloud is not None:
        x0 = min(x0, cloud[:, 0].min() - 0.2)
        x1 = max(x1, cloud[:, 0].max() + 0.2)
        y0 = min(y0, cloud[:, 1].min() - 0.2)
        y1 = max(y1, cloud[:, 1].max() + 0.2)
        zmax = max(zmax, cloud[:, 2].max() + 0.15)

    fig = plt.figure(figsize=(12.8, 6.0), facecolor="#0b0f14")
    axL = fig.add_subplot(121, projection="3d", facecolor="#0b0f14")
    axR = fig.add_subplot(122, projection="3d", facecolor="#0b0f14")
    fig.subplots_adjust(left=0.0, right=1.0, top=0.88, bottom=0.0, wspace=0.02)

    writer = FFMpegWriter(
        fps=fps, bitrate=2800, codec="libx264",
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    cmap = plt.get_cmap("jet")
    wpts = np.asarray(wpts, dtype=np.float32) if wpts is not None else None

    def _style_ax(ax, title):
        ax.set_facecolor("#0b0f14")
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_zlim(0, zmax)
        try:
            ax.set_box_aspect((x1 - x0, y1 - y0, zmax * 1.6))
        except Exception:
            pass
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor("#24303c")
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.label.set_color("#8fa3b8")
            axis.set_tick_params(colors="#5c6f82", labelsize=6)
        ax.set_xlabel("x [m]", fontsize=7)
        ax.set_ylabel("y [m]", fontsize=7)
        ax.set_zlabel("z [m]", fontsize=7)
        ax.set_title(title, color="#8fa3b8", fontsize=9, pad=2)

    with writer.saving(fig, out_path, dpi=110):
        for k, i in enumerate(idxs):
            t, x, y, z, yaw = traj[i]
            for ax in (axL, axR):
                ax.clear()

            # ===== 左：实时点云 =====
            _style_ax(axL, "live point cloud  /lio/cloud_world + heightfield")
            for gz in (0.0, 1.0):
                if gz > zmax:
                    continue
                for gx in np.arange(math.floor(x0), math.ceil(x1) + 1, 1.0):
                    axL.plot([gx, gx], [y0, y1], [gz, gz], color="#1c2630", lw=0.35)
                for gy in np.arange(math.floor(y0), math.ceil(y1) + 1, 1.0):
                    axL.plot([x0, x1], [gy, gy], [gz, gz], color="#1c2630", lw=0.35)
            if surface is not None:
                snorm = np.clip(surface[:, 2] / zmax, 0, 1)
                axL.scatter(surface[:, 0], surface[:, 1], surface[:, 2],
                            c=snorm, cmap=cmap, s=1.0, alpha=0.5,
                            linewidths=0, vmin=0, vmax=1)
            if cloud is not None:
                cnorm = np.clip(cloud[:, 2] / zmax, 0, 1)
                axL.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2],
                            c=cnorm, cmap=cmap, s=1.4, alpha=0.9,
                            linewidths=0, vmin=0, vmax=1)
            tj = np.array(traj[: i + 1])
            axL.plot(tj[:, 1], tj[:, 2], tj[:, 3], color="#00e5ff", lw=1.6, alpha=0.9)
            axL.scatter([x], [y], [z], c="#ffffff", s=40, edgecolors="#00e5ff",
                        linewidths=1.1, depthshade=False)
            axL.quiver(x, y, z, 0.45 * math.cos(yaw), 0.45 * math.sin(yaw), 0,
                       color="#ffd54f", lw=1.8, arrow_length_ratio=0.4)

            # ===== 右：在线体素占据 + 轨迹 =====
            _style_ax(axR, "online voxel occupancy  /grid_map/occupancy_inflate")
            for gy in np.arange(math.floor(y0), math.ceil(y1) + 1, 1.0):
                axR.plot([x0, x1], [gy, gy], [0, 0], color="#1c2630", lw=0.35)
            for gx in np.arange(math.floor(x0), math.ceil(x1) + 1, 1.0):
                axR.plot([gx, gx], [y0, y1], [0, 0], color="#1c2630", lw=0.35)
            occ = occ_snaps[k]
            if occ is not None and len(occ):
                # 按体素自身 z 范围归一→彩虹全谱（对齐真机占据图观感）
                oz0, oz1 = float(occ[:, 2].min()), float(occ[:, 2].max())
                onorm = (occ[:, 2] - oz0) / max(oz1 - oz0, 1e-6)
                axR.scatter(occ[:, 0], occ[:, 1], occ[:, 2],
                            c=onorm, cmap=cmap, s=26, marker="s",
                            alpha=0.95, linewidths=0, vmin=0, vmax=1)
            # 优化轨迹带（紫色）+ 白色航点球
            axR.plot(tj[:, 1], tj[:, 2], tj[:, 3], color="#8e7cc3",
                     lw=7, alpha=0.55, solid_capstyle="round")
            axR.plot(tj[:, 1], tj[:, 2], tj[:, 3], color="#d5c6ff",
                     lw=1.4, alpha=0.9)
            if wpts is not None and len(wpts):
                axR.scatter(wpts[:, 0], wpts[:, 1], wpts[:, 2] - 0.18,
                            c="#ffffff", s=34, depthshade=False,
                            edgecolors="#8e7cc3", linewidths=0.8)
            axR.scatter([x], [y], [z], c="#ffffff", s=55, depthshade=False,
                        edgecolors="#8e7cc3", linewidths=1.4)

            # 相机（慢速环绕，两视图同步）
            elev = 26 + 8 * math.sin(k / max(len(idxs), 1) * math.pi)
            azim = -58 + (k / max(len(idxs) - 1, 1)) * 120
            axL.view_init(elev=elev, azim=azim)
            axR.view_init(elev=elev, azim=azim)

            hud = (f"t={t:5.1f}s   pos=({x:.2f},{y:.2f},{z:.2f})   "
                   f"yaw={math.degrees(yaw):+.0f}°")
            fig.suptitle(
                "MotionSLAM cloud vkernel — cross-floor closed loop "
                "(PCT→BT→SCAN→forwarder)\n" + hud,
                color="#c9d6e2", fontsize=10, family="monospace")
            writer.grab_frame()
            if k % 40 == 0:
                print(f"[render] frame {k}/{len(idxs)}", flush=True)
    plt.close(fig)


def _load_waypoints(yaml_path):
    import yaml

    data = yaml.safe_load(open(yaml_path))
    node = next(iter(data.values()))
    wps = node["ros__parameters"].get("waypoints", [])
    out = []
    for w in wps:
        if isinstance(w, dict):
            out.append((float(w["x"]), float(w["y"]), float(w.get("z", 0.0))))
        else:
            out.append((float(w[0]), float(w[1]), float(w[2]) if len(w) > 2 else 0.0))
    return out


def render(traj, cloud, out_path, fps=15, surface=None):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    n = len(traj)
    step = max(1, math.ceil(n / (fps * 90)))
    idxs = list(range(0, n, step))

    xs = [p[1] for p in traj]
    ys = [p[2] for p in traj]
    zs = [p[3] for p in traj]
    x0, x1 = min(xs) - 1.0, max(xs) + 1.0
    y0, y1 = min(ys) - 1.0, max(ys) + 1.0
    zmax = max(max(zs), 1.2)

    if cloud is not None:
        cx0, cx1 = cloud[:, 0].min(), cloud[:, 0].max()
        cy0, cy1 = cloud[:, 1].min(), cloud[:, 1].max()
        cz1 = cloud[:, 2].max()
        x0, x1 = min(x0, cx0 - 0.3), max(x1, cx1 + 0.3)
        y0, y1 = min(y0, cy0 - 0.3), max(y1, cy1 + 0.3)
        zmax = max(zmax, cz1 + 0.2)

    # dpi=110 → 1276x704，均为偶数（libx264/yuv420p 要求偶数边长）
    fig = plt.figure(figsize=(11.6, 6.4), facecolor="#0b0f14")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0b0f14")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.90, bottom=0.02)

    writer = FFMpegWriter(fps=fps, bitrate=2400, codec="libx264",
                          extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    cmap = plt.get_cmap("jet")

    with writer.saving(fig, out_path, dpi=110):
        for k, i in enumerate(idxs):
            t, x, y, z, yaw = traj[i]
            ax.clear()
            ax.set_facecolor("#0b0f14")

            # 地面网格（两层）
            for gz in (0.0, 1.0):
                if gz > zmax:
                    continue
                for gx in np.arange(math.floor(x0), math.ceil(x1) + 1, 1.0):
                    ax.plot([gx, gx], [y0, y1], [gz, gz],
                            color="#1c2630", lw=0.4, zorder=0)
                for gy in np.arange(math.floor(y0), math.ceil(y1) + 1, 1.0):
                    ax.plot([x0, x1], [gy, gy], [gz, gz],
                            color="#1c2630", lw=0.4, zorder=0)

            # 点云（地表 + 墙/立柱，按高度着色）
            if surface is not None:
                snorm = np.clip(surface[:, 2] / max(zmax, 1e-6), 0, 1)
                ax.scatter(surface[:, 0], surface[:, 1], surface[:, 2],
                           c=snorm, cmap=cmap, s=1.0, alpha=0.45,
                           linewidths=0, vmin=0, vmax=1)
            if cloud is not None:
                cnorm = np.clip(cloud[:, 2] / max(zmax, 1e-6), 0, 1)
                ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2],
                           c=cnorm, cmap=cmap, s=1.4, alpha=0.9,
                           linewidths=0, vmin=0, vmax=1)

            # 轨迹（已走部分）
            tj = np.array(traj[: i + 1])
            ax.plot(tj[:, 1], tj[:, 2], tj[:, 3],
                    color="#00e5ff", lw=1.6, alpha=0.9)

            # 机器人：亮点 + 朝向箭头
            ax.scatter([x], [y], [z], c="#ffffff", s=42, edgecolors="#00e5ff",
                       linewidths=1.2, depthshade=False)
            ax.quiver(x, y, z, 0.45 * math.cos(yaw), 0.45 * math.sin(yaw), 0,
                      color="#ffd54f", lw=1.8, arrow_length_ratio=0.4)

            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
            ax.set_zlim(0, zmax)
            try:
                ax.set_box_aspect((x1 - x0, y1 - y0, zmax * 1.6))
            except Exception:
                pass
            for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
                pane.fill = False
                pane.set_edgecolor("#24303c")
            for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
                axis.label.set_color("#8fa3b8")
                axis.set_tick_params(colors="#5c6f82", labelsize=7)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_zlabel("z [m]")

            # 相机：慢速环绕 + 跟随
            elev = 24 + 10 * math.sin(k / max(len(idxs), 1) * math.pi)
            azim = -60 + (k / max(len(idxs) - 1, 1)) * 140
            ax.view_init(elev=elev, azim=azim)

            hud = (f"t={t:5.1f}s   pos=({x:.2f},{y:.2f},{z:.2f})   "
                   f"yaw={math.degrees(yaw):+.0f}°")
            fig.suptitle(
                "MotionSLAM cloud vkernel — cross-floor closed loop "
                "(PCT→BT→SCAN→forwarder)\n" + hud,
                color="#c9d6e2", fontsize=10, family="monospace")
            writer.grab_frame()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/ws/cross_floor_3d.mp4")
    ap.add_argument("--cloud-topic", default="/lio/cloud_world")
    ap.add_argument("--odom-topic", default="/lio/robo/odom")
    ap.add_argument("--occ-topic", default="/grid_map/occupancy_inflate",
                    help="SCAN 体素占据增量话题（BEST_EFFORT）")
    ap.add_argument("--wp-yaml", default="", help="航点 yaml（右视图画白色航点球）")
    ap.add_argument("--style", choices=["single", "dual"], default="dual",
                    help="single=单视口点云；dual=左实时点云+右体素占据轨迹")
    ap.add_argument("--timeout", type=float, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--map-yaml", default="", help="vkernel 参数 yaml（生成地表点云面）")
    ap.add_argument("--save-npz", default="", help="录制数据保存路径，便于离线反复渲染")
    ap.add_argument("--load-npz", default="", help="从 npz 直接渲染，不再订阅 ROS")
    args = ap.parse_args()

    surface = None
    if args.map_yaml:
        surface = _surface_cloud(_load_map_params(args.map_yaml))
    wpts = _load_waypoints(args.wp_yaml) if args.wp_yaml else None

    if args.load_npz:
        d = np.load(args.load_npz)
        traj = [tuple(r) for r in d["traj"]]
        cloud = d["cloud"]
        occ_rows = []
        if "occ_t" in d.files:
            # 注意：NpzFile.__getitem__ 每次访问都重新读整个数组，
            # 必须先取出再切片，否则 839 次循环重复加载 ~92GB 触发 OOM。
            occ_pts = d["occ_pts"]
            ptr = 0
            for i2, t_val in enumerate(d["occ_t"]):
                n2 = int(d["occ_n"][i2])
                occ_rows.append(
                    (float(t_val), occ_pts[ptr : ptr + n2].reshape(-1, 3)))
                ptr += n2
        if args.style == "dual":
            render_dual(traj, cloud, occ_rows, surface, wpts, args.out, fps=args.fps)
        else:
            render(traj, cloud, args.out, fps=args.fps, surface=surface)
        print(f"WROTE {args.out}")
        return

    if rclpy is None:
        print("需要 ROS 环境（或用 --load-npz 离线渲染）", file=sys.stderr)
        sys.exit(2)
    rclpy.init()
    node = Recorder(args.cloud_topic, args.odom_topic, args.occ_topic)
    import time

    t_deadline = time.time() + args.timeout
    while rclpy.ok() and node.done is None and time.time() < t_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.done is None:
        node.get_logger().warn("timeout, rendering anyway")
    node.get_logger().info(
        f"recorded traj={len(node.traj)} cloud={'yes' if node.cloud is not None else 'no'} "
        f"occ_msgs={len(node.occ_rows)} events={node.events[-5:]}")
    rclpy.shutdown()

    if len(node.traj) < 2 or node.cloud is None:
        print("NOT_ENOUGH_DATA", file=sys.stderr)
        sys.exit(2)
    if args.save_npz:
        occ_t = np.array([t for t, _ in node.occ_rows], dtype=np.float64)
        occ_n = np.array([len(p) for _, p in node.occ_rows], dtype=np.int64)
        occ_pts = (
            np.concatenate([p for _, p in node.occ_rows]).astype(np.float32)
            if node.occ_rows else np.zeros((0, 3), dtype=np.float32)
        )
        np.savez(args.save_npz, traj=np.array(node.traj, dtype=np.float64),
                 cloud=node.cloud.astype(np.float32),
                 occ_t=occ_t, occ_n=occ_n, occ_pts=occ_pts)
    if args.style == "dual":
        render_dual(node.traj, node.cloud, node.occ_rows, surface, wpts,
                    args.out, fps=args.fps)
    else:
        render(node.traj, node.cloud, args.out, fps=args.fps, surface=surface)
    print(f"WROTE {args.out}")


if __name__ == "__main__":
    main()
