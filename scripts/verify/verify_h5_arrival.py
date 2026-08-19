#!/usr/bin/env python3
"""H5 端到端到达 (Nav2 NavigateToPose): 发 goal, 监测机身 odom 是否进入 arrive 半径.

保留供 feature/hybrid-baseline / 历史证据复现；dev Demo 主线请用 verify_scan_planner_h5.py。
需镜像内 ros-humble-nav2-msgs + 完整 Nav2 栈运行中 (/navigate_to_pose)。

默认用 /lio/robo/odom (与 TF world→base_link / robot_pose_viz 一致).
init_pose / ICP 初值 yaw 用 /lio/odom, 见 read_reloc_init_pose.py.

用法 (Nav2 + with_forwarder:=true, 空场):
  python3 scripts/verify/verify_h5_arrival.py --distance 2
  python3 scripts/verify/verify_h5_arrival.py --distance 2 --dry-run
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from h5_arrival_utils import (  # noqa: E402
    ODOM_QOS,
    dist2,
    quat_rpy,
    run_estop_cpp,
    yaw_to_quat,
)


class H5Node(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("verify_h5_arrival")
        self._latest: Odometry | None = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, ODOM_QOS)
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._goal_handle = None

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest = msg

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.05)

    @property
    def latest(self) -> Odometry | None:
        return self._latest

    def send_nav2_goal(self, goal: PoseStamped, wait_server_s: float = 15.0):
        if not self._nav_client.wait_for_server(timeout_sec=wait_server_s):
            raise RuntimeError("/navigate_to_pose action 不可用 (需 Nav2 栈)")
        req = NavigateToPose.Goal()
        req.pose = goal
        send_future = self._nav_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            raise RuntimeError("发送 NavigateToPose goal 超时")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("NavigateToPose goal 被拒绝")
        self._goal_handle = handle
        return handle

    def safe_stop(self) -> None:
        print("  安全停车: cancel goal + 零速...")
        if self._goal_handle is not None:
            try:
                cancel_fut = self._goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=2.0)
            except Exception:
                pass
            self._goal_handle = None
        zero = Twist()
        for _ in range(5):
            self._cmd_pub.publish(zero)
            self.spin_once()
            time.sleep(0.05)
        run_estop_cpp()


def main() -> int:
    parser = argparse.ArgumentParser(description="H5 Nav2 端到端到达验收")
    parser.add_argument("--distance", type=float, default=3.0)
    parser.add_argument("--arrive-dist", type=float, default=0.3)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--odom-topic", default="/lio/robo/odom")
    parser.add_argument("--wait-odom", type=float, default=30.0)
    parser.add_argument("--yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.distance > 3.0:
        print(f"WARN: distance={args.distance}m > 3m, 首轮真机建议 ≤3m", file=sys.stderr)

    rclpy.init()
    node = H5Node(args.odom_topic)
    exit_code = 1

    try:
        print(f"H5 (Nav2): 等待 {args.odom_topic} (最多 {args.wait_odom:.0f}s) ...")
        t0 = time.time()
        while node.latest is None and time.time() - t0 < args.wait_odom:
            node.spin_once()
        if node.latest is None:
            print("ERROR: 无 odom", file=sys.stderr)
            return 1

        ox = node.latest.pose.pose.position.x
        oy = node.latest.pose.pose.position.y
        q = node.latest.pose.pose.orientation
        roll, pitch, yaw_odom = quat_rpy(q.x, q.y, q.z, q.w)
        yaw = yaw_odom + math.radians(args.yaw_offset_deg)

        gx = ox + args.distance * math.cos(yaw)
        gy = oy + args.distance * math.sin(yaw)
        gz = max(node.latest.pose.pose.position.z, 0.0)

        print(
            f"  当前 ({ox:.2f}, {oy:.2f}) "
            f"RPY=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, "
            f"{math.degrees(yaw_odom):.1f})°  [{args.odom_topic}]"
        )
        print(f"  goal ({gx:.2f}, {gy:.2f}, {gz:.2f}) 距离 {args.distance} m")

        if args.dry_run:
            return 0

        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation = yaw_to_quat(yaw)

        print("  发送 NavigateToPose — 需 Nav2 + with_forwarder:=true")
        try:
            handle = node.send_nav2_goal(goal)
        except RuntimeError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 1

        result_future = handle.get_result_async()
        start = time.time()
        best = float("inf")
        nav2_status = "running"

        while time.time() - start < args.timeout:
            node.spin_once()
            if node.latest is not None:
                px = node.latest.pose.pose.position.x
                py = node.latest.pose.pose.position.y
                d_goal = dist2(px, py, gx, gy)
                d_start = dist2(px, py, ox, oy)
                best = min(best, d_goal)
                if d_goal <= args.arrive_dist:
                    elapsed = time.time() - start
                    print(
                        f"OK: 到达 goal 附近, 误差 {d_goal:.3f} m, "
                        f"行走 {d_start:.2f} m, {elapsed:.1f}s"
                    )
                    return 0
            if result_future.done():
                try:
                    wrapped = result_future.result()
                    status = getattr(wrapped, "status", None)
                    nav2_status = f"status={status}"
                    print(f"  Nav2 action 结束 ({nav2_status})")
                except Exception as e:
                    nav2_status = f"exception:{e}"
                    print(f"  Nav2 action 结束 ({nav2_status})")
                break
            time.sleep(0.1)

        px = node.latest.pose.pose.position.x if node.latest else ox
        py = node.latest.pose.pose.position.y if node.latest else oy
        d_goal = dist2(px, py, gx, gy)
        d_start = dist2(px, py, ox, oy)
        print(
            f"FAIL: 未进到达半径, 距 goal {d_goal:.3f} m "
            f"(最近 {best:.3f} m), 行走 {d_start:.2f} m, Nav2={nav2_status}",
            file=sys.stderr,
        )
        exit_code = 1
        return 1
    finally:
        try:
            node.safe_stop()
        except Exception as e:
            print(f"WARN: safe_stop 异常: {e}", file=sys.stderr)
            run_estop_cpp()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
