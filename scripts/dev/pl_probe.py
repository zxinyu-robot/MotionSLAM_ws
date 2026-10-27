#!/usr/bin/env python3
"""P+L 采集探针（测试记录 §6）：旁路订阅，不参与控制。

P-1：/goal_pose 收到 → 其后首条 /global_path 收到（探针本地接收时刻差）。
P-2：/planning/bspline → 其后首条执行速度（mock：SCAN closed_loop_controller 直出 /cmd_vel 给 vkernel；
     真机：/motion/command → forwarder → Sport Move(1008)，两段分别统计）。
频率：bspline / motion_command / sport Move 的平均发布频率。
资源：按进程名关键字 1 Hz 采样 CPU% / RSS。
结果写 JSON（--out），含 n / p50 / p95 / p99 / max。数值是云端 mock 条件下的，不代表真机。
"""
import argparse
import os
import json
import threading
import time

import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from motionslam_msgs.msg import MotionCommand
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from unitree_api.msg import Request

PROC_KEYS = ("scan_planner_node", "pct_resident_planner", "closed_loop_controller", "cmd_vel_forwarder",
             "demo_scan_bt_orchestrator", "vkernel_sim", "terrain_safety_node")


def _proc_key(cmdline):
    # 只看可执行文件 / python 脚本名，避免 launch 参数里的路径误匹配
    names = [os.path.basename(c) for c in cmdline[:2]]
    return next((k for k in PROC_KEYS if any(n.startswith(k) for n in names)), None)


def stats(xs):
    if not xs:
        return {"n": 0}
    s = sorted(xs)

    def q(p):
        return s[min(len(s) - 1, int(round(p * (len(s) - 1))))]
    return {"n": len(s), "p50": q(0.5), "p95": q(0.95), "p99": q(0.99), "max": s[-1]}


class Probe(Node):
    def __init__(self):
        super().__init__("pl_probe")
        self.t0 = time.monotonic()
        self.goal_t = None
        self.p1 = []
        self.bs_pending = []
        self.mc_pending = []
        self.p2_bs_mc = []
        self.p2_bs_cmdvel = []
        self.bs_pending_cv = []
        self.p2_mc_sport = []
        self.counts = {"bspline": 0, "motion_command": 0, "sport_move": 0, "global_path": 0, "goal": 0,
                       "cmd_vel": 0}
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(Path, "/global_path", self.on_path, 10)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bs, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmdvel, 10)
        self.create_subscription(MotionCommand, "/motion/command", self.on_mc, 10)
        self.create_subscription(Request, "/api/sport/request", self.on_sport, qos_profile_sensor_data)

    def on_goal(self, _):
        self.counts["goal"] += 1
        self.goal_t = time.monotonic()

    def on_path(self, _):
        self.counts["global_path"] += 1
        if self.goal_t is not None:
            self.p1.append((time.monotonic() - self.goal_t) * 1e3)
            self.goal_t = None

    def on_bs(self, _):
        self.counts["bspline"] += 1
        self.bs_pending.append(time.monotonic())
        self.bs_pending_cv.append(self.bs_pending[-1])

    def on_cmdvel(self, _):
        self.counts["cmd_vel"] += 1
        if self.bs_pending_cv:
            self.p2_bs_cmdvel.append((time.monotonic() - self.bs_pending_cv[0]) * 1e3)
            self.bs_pending_cv.clear()

    def on_mc(self, _):
        now = time.monotonic()
        self.counts["motion_command"] += 1
        if self.bs_pending:
            self.p2_bs_mc.append((now - self.bs_pending[0]) * 1e3)
            self.bs_pending.clear()
        self.mc_pending.append(now)
        del self.mc_pending[:-50]

    def on_sport(self, m):
        if m.header.identity.api_id != 1008:
            return
        now = time.monotonic()
        self.counts["sport_move"] += 1
        if self.mc_pending:
            self.p2_mc_sport.append((now - self.mc_pending[0]) * 1e3)
            self.mc_pending.clear()


def sample_procs(stop, out):
    procs = {}
    while not stop.is_set():
        for p in psutil.process_iter(["pid", "cmdline", "status"]):
            if p.info["status"] == psutil.STATUS_ZOMBIE:
                continue
            key = _proc_key(p.info["cmdline"] or [])
            if key and p.pid not in procs:
                procs[p.pid] = (key, p)
                p.cpu_percent(None)
        for pid, (key, p) in list(procs.items()):
            try:
                out.setdefault(key, {"cpu": [], "rss_mb": []})
                out[key]["cpu"].append(p.cpu_percent(None))
                out[key]["rss_mb"].append(p.memory_info().rss / 2**20)
            except psutil.Error:
                procs.pop(pid)
        stop.wait(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    n = Probe()
    stop, res = threading.Event(), {}
    th = threading.Thread(target=sample_procs, args=(stop, res), daemon=True)
    th.start()
    end = time.monotonic() + a.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    dur = time.monotonic() - n.t0
    stop.set()
    th.join(timeout=5.0)
    out = {
        "condition": "cloud vkernel mock (no hardware)",
        "duration_s": round(dur, 1),
        "P1_goal_to_global_path_ms": stats(n.p1),
        "P2_bspline_to_cmd_vel_ms": stats(n.p2_bs_cmdvel),
        "P2_bspline_to_motion_command_ms": stats(n.p2_bs_mc),
        "P2_motion_command_to_sport_ms": stats(n.p2_mc_sport),
        "rate_hz": {k: round(v / dur, 2) for k, v in n.counts.items()},
        "counts": n.counts,
        "proc": {k: {"cpu_pct": stats(v["cpu"]), "rss_mb": stats(v["rss_mb"])} for k, v in res.items()},
    }
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    print(json.dumps(out, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
