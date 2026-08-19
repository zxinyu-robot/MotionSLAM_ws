#!/usr/bin/env python3
"""避障链路自检: L1 forwarder + (可选) Nav2 local costmap / cloud_world."""
from __future__ import annotations

import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def run(cmd: list[str]) -> str:
    try:
        return subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True, timeout=8)
    except subprocess.CalledProcessError as e:
        return e.output or ""
    except Exception as e:
        return str(e)


class CloudProbe(Node):
    def __init__(self) -> None:
        super().__init__("check_obstacle_avoid_probe")
        self.got = False
        self.create_subscription(
            PointCloud2,
            "/lio/cloud_world",
            lambda _: setattr(self, "got", True),
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )


def main() -> int:
    rclpy.init()
    print("=== 避障自检 ===")
    nodes = run(["ros2", "node", "list"]).strip().splitlines()
    nodes_raw = [n.strip() for n in nodes if n.strip()]
    node_set = set(nodes_raw)

    ok = True
    if "/cmd_vel_forwarder" not in node_set:
        print("FAIL: 无 /cmd_vel_forwarder → L1 避障未接入")
        print("  启动: ./scripts/motionslam nav start")
        print("  或:   ros2 launch motionslam_bringup mapping_walk.launch.py")
        ok = False
    else:
        backend = run(
            ["ros2", "param", "get", "/cmd_vel_forwarder", "backend"]
        ).strip()
        print(f"  forwarder backend: {backend or 'unknown'}")
        if "obstacles_avoid" not in backend:
            print("WARN: backend 不是 obstacles_avoid，机身 L1 避障可能未启用")
            ok = False
        else:
            print("OK: cmd_vel_forwarder → obstacles_avoid")
        fwd_count = sum(1 for n in nodes_raw if n == "/cmd_vel_forwarder")
        if fwd_count > 1:
            print("WARN: 多个 cmd_vel_forwarder → stop_nav.sh 后只起一个 launch")
            ok = False
        info = run(["ros2", "topic", "info", "/cmd_vel"])
        if "Subscription count: 0" in info or "Subscription count: 0\n" in info:
            print("FAIL: /cmd_vel 无订阅者 → forwarder 未接 cmd_vel")
            ok = False
        elif "cmd_vel_forwarder" not in info and "Subscription count:" in info:
            subs = [
                ln
                for ln in info.splitlines()
                if "Subscription" in ln or "cmd_vel" in ln
            ]
            print(f"  /cmd_vel: {subs[:3]}")

    nav_nodes = {
        "/controller_server",
        "/planner_server",
        "/local_costmap/local_costmap",
        "/lifecycle_manager_navigation",
    }
    missing_nav = nav_nodes - node_set
    if missing_nav:
        print(f"FAIL: Nav2 未齐 (缺 {', '.join(sorted(missing_nav))})")
        ok = False
    else:
        print("OK: Nav2 核心节点在线")
        probe = CloudProbe()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and rclpy.ok():
            rclpy.spin_once(probe, timeout_sec=0.2)
        if probe.got:
            print("OK: 收到 /lio/cloud_world")
        else:
            print("FAIL: 无 /lio/cloud_world → 局部代价图看不到动态障碍")
            print("  确认 relocation_node / super_lio 在跑且 blind≤0.8")
            ok = False
        probe.destroy_node()

    livox_count = sum(1 for n in nodes_raw if n == "/livox_lidar_publisher")
    if livox_count > 1:
        print("WARN: 多个 livox 驱动 → 易占雷达/避障异常，先 stop_nav.sh")
        ok = False

    rclpy.shutdown()
    print("=== 结果:", "PASS" if ok else "FAIL", "===")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
