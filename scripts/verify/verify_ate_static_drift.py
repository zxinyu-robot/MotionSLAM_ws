#!/usr/bin/env python3
"""ATE/RPE 补采 — 静止漂移观测 (无 Nav2 依赖)."""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from h5_arrival_utils import ODOM_QOS, dist2, quat_rpy  # noqa: E402


class OdomProbe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("verify_ate_static_drift")
        self._latest: Odometry | None = None
        self.create_subscription(Odometry, topic, self._cb, ODOM_QOS)

    def _cb(self, msg: Odometry) -> None:
        self._latest = msg

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.05)

    @property
    def latest(self) -> Odometry | None:
        return self._latest


def main() -> int:
    parser = argparse.ArgumentParser(description="静止漂移观测")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--odom-topic", default="/lio/robo/odom")
    args = parser.parse_args()

    rclpy.init()
    node = OdomProbe(args.odom_topic)
    try:
        print(f"static_drift: 等待 odom, 然后静止 {args.duration}s ...")
        t0 = time.time()
        while node.latest is None and time.time() - t0 < 30:
            node.spin_once()
        if node.latest is None:
            return 1

        sx = node.latest.pose.pose.position.x
        sy = node.latest.pose.pose.position.y
        _, _, syaw = quat_rpy(
            node.latest.pose.pose.orientation.x,
            node.latest.pose.pose.orientation.y,
            node.latest.pose.pose.orientation.z,
            node.latest.pose.pose.orientation.w,
        )
        print(f"  t=0: ({sx:.3f}, {sy:.3f}) yaw={math.degrees(syaw):.2f}°")

        start = time.time()
        max_d = 0.0
        max_dyaw = 0.0
        while time.time() - start < args.duration:
            node.spin_once()
            if node.latest is None:
                time.sleep(0.05)
                continue
            px = node.latest.pose.pose.position.x
            py = node.latest.pose.pose.position.y
            _, _, yaw = quat_rpy(
                node.latest.pose.pose.orientation.x,
                node.latest.pose.pose.orientation.y,
                node.latest.pose.pose.orientation.z,
                node.latest.pose.pose.orientation.w,
            )
            max_d = max(max_d, dist2(px, py, sx, sy))
            dyaw = abs(math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw)))
            max_dyaw = max(max_dyaw, dyaw)
            time.sleep(0.1)

        print(
            f"  t={args.duration}s: 最大位移 {max_d:.4f} m, "
            f"最大 yaw 漂移 {math.degrees(max_dyaw):.2f}°"
        )
        print("  ground_truth=无外部真值，仅相对闭环/回测")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
