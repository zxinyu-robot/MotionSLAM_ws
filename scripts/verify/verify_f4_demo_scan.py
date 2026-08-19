#!/usr/bin/env python3
"""F4 Demo: SCAN-Planner 对 goal 产出 /planning/bspline.

用法 (demo_scan_stack 运行中, with_forwarder 可 false):
  python3 scripts/verify_f4_demo_scan.py --once
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from h5_arrival_utils import ODOM_QOS, quat_rpy, yaw_to_quat  # noqa: E402

try:
    from scan_planner_msgs.msg import Bspline
except ImportError:
    Bspline = None  # type: ignore[misc, assignment]


class F4DemoNode(Node):
    def __init__(self) -> None:
        super().__init__("verify_f4_demo_scan")
        self._odom: Odometry | None = None
        self._bspline_count = 0
        self.create_subscription(Odometry, "/lio/robo/odom", self._odom_cb, ODOM_QOS)
        if Bspline is not None:
            self.create_subscription(
                Bspline,
                "/planning/bspline",
                lambda _m: setattr(self, "_bspline_count", self._bspline_count + 1),
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5,
                ),
            )
        self._goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.05)

    def publish_goal_ahead(self, distance: float) -> None:
        if self._odom is None:
            raise RuntimeError("无 /lio/robo/odom")
        ox = self._odom.pose.pose.position.x
        oy = self._odom.pose.pose.position.y
        q = self._odom.pose.pose.orientation
        _r, _p, yaw = quat_rpy(q.x, q.y, q.z, q.w)
        gx = ox + distance * math.cos(yaw)
        gy = oy + distance * math.sin(yaw)
        gz = max(self._odom.pose.pose.position.z, 0.3)
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation = yaw_to_quat(yaw)
        for _ in range(3):
            self._goal_pub.publish(goal)
            self.spin_once()
            time.sleep(0.05)


def main() -> int:
    parser = argparse.ArgumentParser(description="F4 Demo SCAN bspline 检查")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--distance", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--wait-odom", type=float, default=20.0)
    args = parser.parse_args()

    if Bspline is None:
        print(
            "ERROR: 无 scan_planner_msgs (请 source scan_planner_ws/install/setup.bash)",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = F4DemoNode()

    t0 = time.time()
    while node._odom is None and time.time() - t0 < args.wait_odom:
        node.spin_once()

    if node._odom is None:
        print("ERROR: 无 /lio/robo/odom", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    before = node._bspline_count
    try:
        node.publish_goal_ahead(args.distance)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    deadline = time.time() + args.timeout
    while time.time() < deadline and node._bspline_count <= before:
        node.spin_once()
        time.sleep(0.05)

    got = node._bspline_count > before
    print(
        f"F4 Demo: /planning/bspline {'收到' if got else '未收到'} "
        f"(Δ={node._bspline_count - before}, timeout={args.timeout}s)"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if got else 1


if __name__ == "__main__":
    sys.exit(main())
