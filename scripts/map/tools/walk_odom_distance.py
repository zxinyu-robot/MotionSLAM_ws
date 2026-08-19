#!/usr/bin/env python3
"""沿 /lio/odom 累计位移慢走，用于建图/后端联调。

需要 cmd_vel_forwarder 已起，且 **backend=obstacles_avoid**（机身 L1 避障）。
勿用 sport 测距（仅 Sport+FreeAvoid，App 常显示避障未开、近障弱）。

  ros2 run motionslam_pipeline cmd_vel_forwarder --ros-args \
    --params-file .../pipeline.yaml
  # pipeline.yaml 默认 backend: obstacles_avoid
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WalkOdom(Node):
    def __init__(self, distance_m: float, speed_mps: float, odom_topic: str) -> None:
        super().__init__("walk_odom_distance")
        self.distance_m = distance_m
        self.speed_mps = max(0.05, min(speed_mps, 0.32))
        self.x0: float | None = None
        self.y0: float | None = None
        self.x = 0.0
        self.y = 0.0
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_timer(0.05, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.x0 is None:
            self.x0 = self.x
            self.y0 = self.y

    def traveled(self) -> float:
        if self.x0 is None:
            return 0.0
        return math.hypot(self.x - self.x0, self.y - self.y0)

    def _tick(self) -> None:
        t = Twist()
        if self.traveled() >= self.distance_m:
            self.pub.publish(t)
            print(f"OK: traveled {self.traveled():.2f} m (target {self.distance_m} m)")
            rclpy.shutdown()
            return
        t.linear.x = self.speed_mps
        self.pub.publish(t)


def estop() -> None:
    subprocess.run(
        ["ros2", "run", "motionslam_pipeline", "estop_go2"],
        check=False,
        timeout=8,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=3.0)
    parser.add_argument("--speed", type=float, default=0.25, help="m/s, 建图门控建议 <0.35")
    parser.add_argument("--odom-topic", default="/lio/odom")
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    rclpy.init()
    node = WalkOdom(args.distance, args.speed, args.odom_topic)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.x0 is not None and node.traveled() >= args.distance:
                break
    finally:
        estop()
        node.destroy_node()
        rclpy.shutdown()

    if node.x0 is None:
        print("FAIL: no odom", file=sys.stderr)
        return 1
    if node.traveled() < args.distance * 0.85:
        print(
            f"FAIL: only {node.traveled():.2f} m in {args.timeout}s",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
