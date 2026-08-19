#!/usr/bin/env python3
"""T2.5 外参辅助: 对比 /lio/robo/odom 与 /utlidar/robot_odom 的 roll/pitch/yaw.

用法 (容器内, 栈运行中):
  python3 scripts/verify/calibrate_odom_robo.py --once
  python3 scripts/verify/calibrate_odom_robo.py --once --report-only   # e2e_verify 用

NOTE: 输出仅为观测/建议初值, 须真机迭代写入 super_lio_mid360.yaml 的 odom_robo 后复测。
详见 docs/开发参考/T2.5_odom_robo外参标定.md
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def sample_odom(node: Node, topic: str, timeout: float = 3.0) -> Odometry | None:
    msg_holder: list[Odometry] = []

    def cb(m: Odometry) -> None:
        msg_holder.append(m)

    node.create_subscription(Odometry, topic, cb, 10)
    import time

    t0 = time.time()
    while not msg_holder and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    return msg_holder[0] if msg_holder else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--once", action="store_true", help="采一次并退出")
    parser.add_argument("--report-only", action="store_true", help="仅报告 pitch 差, exit 0/1")
    args = parser.parse_args()

    rclpy.init()
    node = Node("calibrate_odom_robo")

    lio = sample_odom(node, "/lio/robo/odom") or sample_odom(node, "/lio/odom")
    ut = sample_odom(node, "/utlidar/robot_odom")

    if not lio:
        print("ERROR: 无 /lio/robo/odom 或 /lio/odom", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    q = lio.pose.pose.orientation
    lr, lp, ly = quat_to_rpy(q.x, q.y, q.z, q.w)
    print(f"LIO robo/odom: roll={lr:.2f} pitch={lp:.2f} yaw={ly:.2f}")

    if not ut:
        print("WARN: 无 /utlidar/robot_odom, 无法对比")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    q2 = ut.pose.pose.orientation
    ur, up, uy = quat_to_rpy(q2.x, q2.y, q2.z, q2.w)
    print(f"Unitree odom:  roll={ur:.2f} pitch={up:.2f} yaw={uy:.2f}")
    dp = lp - up
    print(f"pitch 差 (LIO - Unitree): {dp:.2f} deg")
    # Super-LIO: robo_state.R = R_world_imu * g_odom_robo.R_; config pitch 增大则 /lio/robo/odom pitch 减小
    if abs(dp) < 3.0:
        print("OK: pitch 差 < 3° (F1 通过)")
    else:
        print(f"建议: odom_robo pitch 在 yaml 当前值上 **加** {dp:.1f} deg 后重启 LIO 复测")
        print(f"      (若 yaml 为全 0 初值, 可直接设 pitch ≈ {dp:.1f} deg)")

    node.destroy_node()
    rclpy.shutdown()

    if args.report_only:
        return 0 if abs(dp) < 3.0 else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
