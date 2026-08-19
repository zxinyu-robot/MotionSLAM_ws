#!/usr/bin/env python3
"""ATE/RPE 补采 — 空场直线往返 3m.

去程 + 180° 调头 + 回程到起点附近; 输出起终点坐标供 evo.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped

_script_dir = os.path.dirname(os.path.abspath(__file__))
_root = os.path.dirname(_script_dir)
for _p in (_root, _script_dir):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from verify_h5_arrival import H5Node, dist2, quat_rpy, yaw_to_quat  # noqa: E402


def send_and_wait(
    node: H5Node,
    gx: float,
    gy: float,
    gz: float,
    yaw: float,
    arrive_dist: float,
    timeout: float,
    label: str,
) -> tuple[int, float, float]:
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.position.z = gz
    goal.pose.orientation = yaw_to_quat(yaw)

    print(f"  [{label}] goal ({gx:.2f}, {gy:.2f}) yaw={math.degrees(yaw):.1f}°")
    handle = node.send_nav2_goal(goal)
    result_future = handle.get_result_async()
    start = time.time()

    while time.time() - start < timeout:
        node.spin_once()
        if node.latest is None:
            time.sleep(0.05)
            continue
        px = node.latest.pose.pose.position.x
        py = node.latest.pose.pose.position.y
        if dist2(px, py, gx, gy) <= arrive_dist:
            print(f"  [{label}] OK 误差 {dist2(px, py, gx, gy):.3f} m, {time.time()-start:.1f}s")
            return 0, px, py
        if result_future.done():
            break
        time.sleep(0.1)

    px = node.latest.pose.pose.position.x if node.latest else gx
    py = node.latest.pose.pose.position.y if node.latest else gy
    d = dist2(px, py, gx, gy)
    print(f"  [{label}] FAIL 距 goal {d:.3f} m", file=sys.stderr)
    return 1, px, py


def main() -> int:
    parser = argparse.ArgumentParser(description="ATE 空场 3m 往返")
    parser.add_argument("--leg-distance", type=float, default=3.0)
    parser.add_argument("--arrive-dist", type=float, default=0.35)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--odom-topic", default="/lio/robo/odom")
    args = parser.parse_args()

    rclpy.init()
    node = H5Node(args.odom_topic)
    rc = 1
    try:
        print("ATE straight_3m: 等待 odom ...")
        t0 = time.time()
        while node.latest is None and time.time() - t0 < 30:
            node.spin_once()
        if node.latest is None:
            return 1

        ox = node.latest.pose.pose.position.x
        oy = node.latest.pose.pose.position.y
        _, _, yaw0 = quat_rpy(
            node.latest.pose.pose.orientation.x,
            node.latest.pose.pose.orientation.y,
            node.latest.pose.pose.orientation.z,
            node.latest.pose.pose.orientation.w,
        )
        gz = max(node.latest.pose.pose.position.z, 0.0)
        print(f"  起点 ({ox:.2f}, {oy:.2f}) yaw={math.degrees(yaw0):.1f}°")

        gx = ox + args.leg_distance * math.cos(yaw0)
        gy = oy + args.leg_distance * math.sin(yaw0)

        rc1, _, _ = send_and_wait(
            node, gx, gy, gz, yaw0, args.arrive_dist, args.timeout, "去程"
        )
        if rc1 != 0:
            return 1

        time.sleep(2.0)
        node.spin_once()
        if node.latest is None:
            return 1
        cx = node.latest.pose.pose.position.x
        cy = node.latest.pose.pose.position.y
        _, _, yaw_now = quat_rpy(
            node.latest.pose.pose.orientation.x,
            node.latest.pose.pose.orientation.y,
            node.latest.pose.pose.orientation.z,
            node.latest.pose.pose.orientation.w,
        )
        yaw_back = yaw_now + math.pi

        rc2, fx, fy = send_and_wait(
            node, ox, oy, gz, yaw_back, args.arrive_dist, args.timeout, "回程"
        )
        loop_err = dist2(fx, fy, ox, oy)
        print(f"  闭环误差 (回起点): {loop_err:.3f} m")
        print(f"  ground_truth=无外部真值，仅相对闭环/回测")
        rc = 0 if rc2 == 0 else 1
        return rc
    finally:
        try:
            node.safe_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
