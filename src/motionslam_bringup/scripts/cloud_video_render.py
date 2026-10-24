#!/usr/bin/env python3
"""云端闭环视频录制渲染：订阅 /lio/robo/odom + /demo/mission/event，
mission_succeeded/failed 或超时后用 matplotlib 渲染 mp4（俯视 + 侧视爬升）。

用法（容器内）:
    python3 cloud_video_render.py --map-yaml /ws/src/motionslam_bringup/config/vkernel_multifloor.yaml \
        --out /ws/cross_floor_demo.mp4 --timeout 360
"""
import argparse
import json
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self):
        super().__init__("cloud_video_render")
        self.traj = []  # (t, x, y, z)
        self.events = []
        self.done = None
        self.create_subscription(Odometry, "/lio/robo/odom", self._on_odom, 10)
        self.create_subscription(String, "/demo/mission/event", self._on_event, 10)
        self._t0 = None

    def _on_odom(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        p = msg.pose.pose.position
        self.traj.append((now - self._t0, p.x, p.y, p.z))

    def _on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except Exception:
            return
        name = ev.get("event", "")
        self.events.append(name)
        self.get_logger().info(f"[event] {name} idx={ev.get('subgoal_index')}")
        if name in ("mission_succeeded", "mission_failed", "mission_rejected"):
            self.done = name


def _load_map(yaml_path):
    import yaml

    data = yaml.safe_load(open(yaml_path))
    params = data["vkernel_sim"]["ros__parameters"]

    def chunks(flat, n):
        return [flat[i : i + n] for i in range(0, len(flat) - n + 1, n)]

    walls = chunks(params.get("walls", []), 5)
    floors = chunks(params.get("floors", []), 5)
    ramps = chunks(params.get("ramps", []), 7)
    pillars = chunks(params.get("pillars", []), 3)
    return walls, floors, ramps, pillars


def render(traj, map_yaml, out_path, fps=15):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    from matplotlib.patches import Rectangle, Circle, Polygon

    walls, floors, ramps, pillars = _load_map(map_yaml)
    xs = [p[1] for p in traj] + [w[0] for w in walls] + [w[2] for w in walls]
    ys = [p[2] for p in traj] + [w[1] for w in walls] + [w[3] for w in walls]
    x0, x1 = min(xs) - 0.6, max(xs) + 0.6
    y0, y1 = min(ys) - 0.6, max(ys) + 0.6

    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=(13, 5.4), gridspec_kw={"width_ratios": [1.35, 1]}
    )
    fig.suptitle("MotionSLAM cloud vkernel cross-floor closed loop (PCT→BT→SCAN→forwarder)", fontsize=13)

    # ---- 俯视 ----
    ax.set_title("Top view (x-y)")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    for x1f, y1f, x2f, y2f, z in floors:
        ax.add_patch(
            Rectangle((x1f, y1f), x2f - x1f, y2f - y1f, fc="#cfe3ff", alpha=0.35, zorder=0)
        )
        ax.text((x1f + x2f) / 2, y2f - 0.35, f"floor z={z:.1f}", ha="center", fontsize=9, color="#2255aa")
    for x1r, y1r, x2r, y2r, z1r, z2r, axis in ramps:
        ax.add_patch(
            Rectangle((x1r, y1r), x2r - x1r, y2r - y1f if False else y2r - y1r, fc="#ffe0b3", alpha=0.45, zorder=0)
        )
        ax.text((x1r + x2r) / 2, (y1r + y2r) / 2, f"ramp {z1r:.1f}→{z2r:.1f}", ha="center", va="center", fontsize=9, color="#996300")
    for x1w, y1w, x2w, y2w, z0 in walls:
        ax.plot([x1w, x2w], [y1w, y2w], color="#333333", lw=2.5, solid_capstyle="butt")
    for cx, cy, r in pillars:
        ax.add_patch(Circle((cx, cy), r, fc="#8855cc", alpha=0.7))
    (trail,) = ax.plot([], [], color="#dd2222", lw=2, zorder=5)
    (robot,) = ax.plot([], [], marker="o", ms=10, mfc="#22aa55", mec="k", zorder=6)
    floor_lbl = ax.text(x0 + 0.15, y1 - 0.4, "", fontsize=10, color="#222")

    # ---- 侧视 (x-z 爬升) ----
    ax2.set_title("Side view (x-z climb)")
    ax2.set_xlim(x0, x1)
    zmax = max(1.6, max(p[3] for p in traj) + 0.4)
    ax2.set_ylim(-0.15, zmax)
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("z [m]")
    for x1f, y1f, x2f, y2f, z in floors:
        ax2.add_patch(Rectangle((x1f, -0.05), x2f - x1f, 0.08, fc="#cfe3ff", alpha=0.9, zorder=1))
        ax2.text((x1f + x2f) / 2, z + 0.06, f"z={z:.1f}", ha="center", fontsize=9, color="#2255aa")
    for x1r, y1r, x2r, y2r, z1r, z2r, axis in ramps:
        ax2.add_patch(Polygon([(x1r, z1r), (x2r, z2r), (x2r, z2r - 0.06), (x1r, z1r - 0.06)], fc="#ffe0b3", alpha=0.95, zorder=1))
        ax2.plot([x1r, x2r], [z1r, z2r], color="#996300", lw=1.5, ls="--")
    (trail2,) = ax2.plot([], [], color="#dd2222", lw=2)
    (robot2,) = ax2.plot([], [], marker="o", ms=9, mfc="#22aa55", mec="k")
    time_lbl = ax2.text(x0 + 0.15, zmax - 0.15, "", fontsize=10, color="#222")

    fig.tight_layout(rect=[0, 0, 1, 0.94])

    writer = FFMpegWriter(fps=fps, metadata={"title": "motionslam cloud multifloor"})
    n = len(traj)
    step = max(1, math.ceil(n / (fps * 90)))  # 最长 ~90s
    frames = list(range(0, n, step))
    with writer.saving(fig, out_path, dpi=130):
        for i in frames:
            t, x, y, z = traj[i]
            tr = [(p[1], p[2]) for p in traj[: i + 1]]
            trail.set_data([p[0] for p in tr], [p[1] for p in tr])
            robot.set_data([x], [y])
            tr2 = [(p[1], p[3]) for p in traj[: i + 1]]
            trail2.set_data([p[0] for p in tr2], [p[1] for p in tr2])
            robot2.set_data([x], [z])
            floor_lbl.set_text(f"t={t:6.1f}s  pos=({x:5.2f},{y:5.2f}) z={z:4.2f}")
            time_lbl.set_text(f"t={t:6.1f}s")
            writer.grab_frame()
    print(f"[render] {len(frames)} frames -> {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-yaml", required=True)
    ap.add_argument("--out", default="/ws/cross_floor_demo.mp4")
    ap.add_argument("--timeout", type=float, default=360.0)
    ap.add_argument("--fps", type=int, default=15)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder()
    import time

    t0 = time.monotonic()
    while node.done is None and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    result = node.done or "timeout"
    traj = list(node.traj)
    node.destroy_node()
    rclpy.shutdown()
    print(f"[record] result={result} poses={len(traj)} events={len(node.events)}")
    if len(traj) < 10:
        print("[render] not enough data")
        sys.exit(2)
    render(traj, args.map_yaml, args.out, args.fps)
    sys.exit(0 if result == "mission_succeeded" else 1)


if __name__ == "__main__":
    main()
