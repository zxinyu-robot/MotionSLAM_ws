#!/usr/bin/env python3
"""F3 Demo: /lio/cloud_world 障碍高度带内有点 (Super-LIO + SCAN 栈).

用法 (demo_scan_stack / mapping + mid360):
  python3 scripts/verify_f3_demo_cloud.py --once
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudProbe(Node):
    def __init__(self) -> None:
        super().__init__("verify_f3_demo_cloud")
        self._count = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.create_subscription(PointCloud2, "/lio/cloud_world", self._cb, qos)

    def _cb(self, msg: PointCloud2) -> None:
        z_min = self.get_parameter("z_min").value
        z_max = self.get_parameter("z_max").value
        n = 0
        for _x, _y, z in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            if z_min <= float(z) <= z_max:
                n += 1
        self._count = max(self._count, n)

    @property
    def count(self) -> int:
        return self._count


def main() -> int:
    parser = argparse.ArgumentParser(description="F3 Demo cloud_world 检查")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--min-points", type=int, default=80)
    parser.add_argument("--z-min", type=float, default=0.08)
    parser.add_argument("--z-max", type=float, default=1.2)
    args = parser.parse_args()

    rclpy.init()
    node = CloudProbe()
    node.declare_parameter("z_min", args.z_min)
    node.declare_parameter("z_max", args.z_max)

    deadline = time.time() + args.timeout
    while time.time() < deadline and node.count < args.min_points:
        rclpy.spin_once(node, timeout_sec=0.2)

    ok = node.count >= args.min_points
    print(
        f"F3 Demo: cloud_world 障碍带 [{args.z_min},{args.z_max}]m "
        f"点数={node.count} (需>={args.min_points}) → {'PASS' if ok else 'FAIL'}"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
