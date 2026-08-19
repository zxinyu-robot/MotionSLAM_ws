#!/usr/bin/env python3
"""F2 定位漂移验收: 静止漂移 + 直线 5m 回测.

用法 (容器内, Super-LIO 运行中):
  python3 scripts/verify_f2_drift.py --static --duration 60
  python3 scripts/verify_f2_drift.py --line --target-m 5.0 --return-tol 0.15

通过标准 (E2E F2):
  静止 duration 内 /lio/odom 位置漂移 < 0.05 m
  直线 target-m 行走后回到起点误差 < return-tol m
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def dist3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def dist2(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sample_odom(node: Node, topic: str, timeout: float = 5.0) -> Odometry | None:
    holder: list[Odometry] = []

    def cb(m: Odometry) -> None:
        holder.append(m)

    node.create_subscription(Odometry, topic, cb, 10)
    t0 = time.time()
    while not holder and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    return holder[0] if holder else None


def pos_from(msg: Odometry) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    return p.x, p.y, p.z


def speed_from(msg: Odometry) -> float:
    v = msg.twist.twist.linear
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class OdomMonitor:
    def __init__(self, node: Node, topic: str) -> None:
        self._node = node
        self._latest: Odometry | None = None
        node.create_subscription(Odometry, topic, self._on_odom, 50)

    def _on_odom(self, msg: Odometry) -> None:
        self._latest = msg

    def spin_once(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0.05)

    @property
    def latest(self) -> Odometry | None:
        return self._latest


def resolve_odom_topic(node: Node) -> str:
    for topic in ("/lio/odom", "/lio/robo/odom"):
        if sample_odom(node, topic, timeout=2.0):
            return topic
    return ""


def run_static(node: Node, topic: str, duration: float, max_drift: float) -> int:
    mon = OdomMonitor(node, topic)
    print(f"F2-static: 订阅 {topic}, 请保持狗静止 {duration:.0f}s ...")
    t0 = time.time()
    while mon.latest is None and time.time() - t0 < 5.0:
        mon.spin_once()
    if mon.latest is None:
        print("ERROR: 无 odom 数据", file=sys.stderr)
        return 1

    origin = pos_from(mon.latest)
    max_d = 0.0
    max_speed = 0.0
    end = time.time() + duration
    while time.time() < end:
        mon.spin_once()
        if mon.latest is None:
            continue
        p = pos_from(mon.latest)
        max_d = max(max_d, dist3(origin, p))
        max_speed = max(max_speed, speed_from(mon.latest))
        time.sleep(0.1)

    print(f"  起点 ({origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f})")
    print(f"  最大漂移: {max_d:.4f} m (限 {max_drift} m)")
    print(f"  最大线速: {max_speed:.3f} m/s")
    if max_d <= max_drift:
        print("OK: F2 静止漂移通过")
        return 0
    print("FAIL: F2 静止漂移超限", file=sys.stderr)
    return 1


def run_line(
    node: Node,
    topic: str,
    target_m: float,
    return_tol: float,
    leg_timeout: float,
) -> int:
    mon = OdomMonitor(node, topic)
    print(f"F2-line: 订阅 {topic}")
    print(f"  1) 遥控狗沿直线行走约 {target_m} m ...")
    t0 = time.time()
    while mon.latest is None and time.time() - t0 < 5.0:
        mon.spin_once()
    if mon.latest is None:
        print("ERROR: 无 odom 数据", file=sys.stderr)
        return 1

    start = pos_from(mon.latest)
    start_xy = (start[0], start[1])
    max_xy = 0.0
    leg_done = False
    t_leg = time.time()
    while time.time() - t_leg < leg_timeout:
        mon.spin_once()
        if mon.latest is None:
            continue
        p = pos_from(mon.latest)
        d = dist2(start_xy, (p[0], p[1]))
        max_xy = max(max_xy, d)
        if d >= target_m * 0.9:
            leg_done = True
            print(f"  已行走 {d:.2f} m (目标 {target_m} m)")
            break
        time.sleep(0.05)

    if not leg_done:
        print(f"FAIL: {leg_timeout}s 内未达到 {target_m} m (最大 {max_xy:.2f} m)", file=sys.stderr)
        return 1

    print("  2) 请沿原路返回起点 ...")
    t_ret = time.time()
    returned = False
    while time.time() - t_ret < leg_timeout:
        mon.spin_once()
        if mon.latest is None:
            continue
        p = pos_from(mon.latest)
        err = dist2(start_xy, (p[0], p[1]))
        if err <= return_tol:
            returned = True
            print(f"  回测误差: {err:.4f} m (限 {return_tol} m)")
            break
        time.sleep(0.05)

    if not returned:
        p = pos_from(mon.latest) if mon.latest else start
        err = dist2(start_xy, (p[0], p[1]))
        print(f"FAIL: 回测超时, 距起点 {err:.3f} m", file=sys.stderr)
        return 1

    print("OK: F2 直线回测通过")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="F2 定位漂移验收")
    parser.add_argument("--static", action="store_true", help="静止漂移测试")
    parser.add_argument("--line", action="store_true", help="直线 5m 回测")
    parser.add_argument("--duration", type=float, default=60.0, help="静止时长 (s)")
    parser.add_argument("--max-drift", type=float, default=0.05, help="静止最大漂移 (m)")
    parser.add_argument("--target-m", type=float, default=5.0, help="直线目标距离 (m)")
    parser.add_argument("--return-tol", type=float, default=0.15, help="回起点误差 (m)")
    parser.add_argument("--leg-timeout", type=float, default=180.0, help="单程超时 (s)")
    parser.add_argument("--odom-topic", default="", help="默认 /lio/odom")
    args = parser.parse_args()

    if not args.static and not args.line:
        parser.error("指定 --static 或 --line")

    rclpy.init()
    node = Node("verify_f2_drift")
    topic = args.odom_topic or resolve_odom_topic(node)
    if not topic:
        print("ERROR: 无 /lio/odom", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    rc = 0
    if args.static:
        rc = run_static(node, topic, args.duration, args.max_drift)
    if rc == 0 and args.line:
        rc = run_line(node, topic, args.target_m, args.return_tol, args.leg_timeout)

    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
