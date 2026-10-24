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
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def _decode_xyz(msg: PointCloud2):
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
    def __init__(self, cloud_topic, odom_topic):
        super().__init__("cloud_video_render_3d")
        self.traj = []  # (t, x, y, z, yaw)
        self.cloud = None  # 最新一帧点云 Nx3
        self.events = []
        self.done = None
        self._t0 = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos)
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


def render(traj, cloud, out_path, fps=15):
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

            # 点云（全量，按高度着色）
            if cloud is not None:
                cnorm = np.clip(cloud[:, 2] / max(zmax, 1e-6), 0, 1)
                ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2],
                           c=cnorm, cmap=cmap, s=1.2, alpha=0.85,
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
    ap.add_argument("--timeout", type=float, default=480)
    ap.add_argument("--fps", type=int, default=15)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.cloud_topic, args.odom_topic)
    import time

    t_deadline = time.time() + args.timeout
    while rclpy.ok() and node.done is None and time.time() < t_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.done is None:
        node.get_logger().warn("timeout, rendering anyway")
    node.get_logger().info(
        f"recorded traj={len(node.traj)} cloud={'yes' if node.cloud is not None else 'no'} "
        f"events={node.events[-5:]}")
    rclpy.shutdown()

    if len(node.traj) < 2 or node.cloud is None:
        print("NOT_ENOUGH_DATA", file=sys.stderr)
        sys.exit(2)
    render(node.traj, node.cloud, args.out, fps=args.fps)
    print(f"WROTE {args.out}")


if __name__ == "__main__":
    main()
