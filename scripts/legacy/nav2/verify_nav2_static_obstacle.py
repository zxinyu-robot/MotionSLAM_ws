#!/usr/bin/env python3
"""NAV-2 静态障碍验收: 正前方 NavigateToPose, 判定 到达 / 绕行 / 停住 / 试探失败.

请在路径约 1.0–2.0 m 处放置纸箱 (离地 0.2–1.5 m). 空场则等同 H5.

用法 (nav_stack + with_forwarder:=true):
  python3 scripts/verify_nav2_static_obstacle.py --distance 3
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

# 复用 H5 工具
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from verify_h5_arrival import (  # noqa: E402
    H5Node,
    dist2,
    quat_rpy,
    yaw_to_quat,
)


def lateral_offset(sx: float, sy: float, gx: float, gy: float, px: float, py: float) -> float:
    dx, dy = gx - sx, gy - sy
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return 0.0
    nx, ny = -dy / L, dx / L
    return abs((px - sx) * nx + (py - sy) * ny)


def main() -> int:
    parser = argparse.ArgumentParser(description="NAV-2 静态障碍")
    parser.add_argument("--distance", type=float, default=3.0)
    parser.add_argument("--arrive-dist", type=float, default=0.3)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--odom-topic", default="/lio/robo/odom")
    parser.add_argument("--wait-odom", type=float, default=30.0)
    parser.add_argument(
        "--detour-lateral-m",
        type=float,
        default=0.35,
        help="侧向偏移超过此值且前进>1m 视为绕行",
    )
    parser.add_argument(
        "--stall-speed",
        type=float,
        default=0.04,
        help="线速度低于此视为静止",
    )
    parser.add_argument(
        "--stall-sec",
        type=float,
        default=8.0,
        help="持续静止且未到达 goal 视为「停住」(PASS)",
    )
    parser.add_argument(
        "--probe-window",
        type=float,
        default=25.0,
        help="试探判定窗口 (秒)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = H5Node(args.odom_topic)
    last_cmd = Twist()
    node.create_subscription(Twist, "/cmd_vel", lambda m: setattr(node, "_last_cmd", m), 10)
    node._last_cmd = last_cmd  # type: ignore[attr-defined]

    try:
        print("NAV-2: 等待 odom ...")
        t0 = time.time()
        while node.latest is None and time.time() - t0 < args.wait_odom:
            node.spin_once()
        if node.latest is None:
            print("ERROR: 无 odom", file=sys.stderr)
            return 1

        ox = node.latest.pose.pose.position.x
        oy = node.latest.pose.pose.position.y
        q = node.latest.pose.pose.orientation
        _, _, yaw = quat_rpy(q.x, q.y, q.z, q.w)
        gx = ox + args.distance * math.cos(yaw)
        gy = oy + args.distance * math.sin(yaw)
        gz = max(node.latest.pose.pose.position.z, 0.0)

        print(f"  起点 ({ox:.2f}, {oy:.2f}) → goal ({gx:.2f}, {gy:.2f}), {args.distance} m")
        print("  若测纸箱: 请放在狗头正前方约 1–2 m, 高度 0.2–1.5 m")
        print("  发送 NavigateToPose ...")

        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation = yaw_to_quat(yaw)

        handle = node.send_nav2_goal(goal)
        result_future = handle.get_result_async()
        start = time.time()
        max_lat = 0.0
        max_travel = 0.0
        stall_t0: float | None = None
        probe_pulses = 0
        last_vx_sign = 0
        window_start = start
        window_start_progress = 0.0

        while time.time() - start < args.timeout:
            node.spin_once()
            now = time.time()
            if node.latest is None:
                time.sleep(0.05)
                continue

            px = node.latest.pose.pose.position.x
            py = node.latest.pose.pose.position.y
            travel = dist2(px, py, ox, oy)
            d_goal = dist2(px, py, gx, gy)
            max_travel = max(max_travel, travel)
            max_lat = max(max_lat, lateral_offset(ox, oy, gx, gy, px, py))

            cmd: Twist = getattr(node, "_last_cmd", Twist())
            speed = math.hypot(cmd.linear.x, cmd.linear.y)

            if d_goal <= args.arrive_dist:
                print(
                    f"PASS [到达]: 误差 {d_goal:.3f} m, 行走 {travel:.2f} m, "
                    f"最大侧偏 {max_lat:.2f} m, {now - start:.1f}s"
                )
                return 0

            if speed < args.stall_speed:
                if stall_t0 is None:
                    stall_t0 = now
                elif now - stall_t0 >= args.stall_sec and travel > 0.4 and d_goal > args.arrive_dist + 0.5:
                    print(
                        f"PASS [停住]: 距 goal {d_goal:.2f} m, 行走 {travel:.2f} m, "
                        f"静止 ≥{args.stall_sec:.0f}s, 侧偏 {max_lat:.2f} m"
                    )
                    return 0
            else:
                stall_t0 = None

            vx = cmd.linear.x
            if abs(vx) > 0.08:
                sign = 1 if vx > 0 else -1
                if last_vx_sign != 0 and sign != last_vx_sign:
                    probe_pulses += 1
                last_vx_sign = sign

            if now - window_start > args.probe_window:
                progress = travel - window_start_progress
                if probe_pulses >= 8 and progress < 0.35:
                    print(
                        f"FAIL [试探]: {args.probe_window:.0f}s 内脉冲 {probe_pulses} 次, "
                        f"净前进 {progress:.2f} m, 距 goal {d_goal:.2f} m",
                        file=sys.stderr,
                    )
                    return 2
                window_start = now
                window_start_progress = travel
                probe_pulses = 0

            if max_lat >= args.detour_lateral_m and travel >= 1.0 and d_goal < args.distance - 0.3:
                print(
                    f"PASS [绕行]: 侧偏 {max_lat:.2f} m, 行走 {travel:.2f} m, "
                    f"距 goal {d_goal:.2f} m, {now - start:.1f}s"
                )
                return 0

            if result_future.done():
                break
            time.sleep(0.1)

        px = node.latest.pose.pose.position.x if node.latest else ox
        py = node.latest.pose.pose.position.y if node.latest else oy
        d_goal = dist2(px, py, gx, gy)
        print(
            f"FAIL [超时/中止]: 距 goal {d_goal:.2f} m, 行走 {max_travel:.2f} m, "
            f"侧偏 {max_lat:.2f} m",
            file=sys.stderr,
        )
        return 1
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
