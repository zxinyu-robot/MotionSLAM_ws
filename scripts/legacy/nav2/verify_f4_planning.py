#!/usr/bin/env python3
"""F4 规划可行: 空场发 goal, 检查 Nav2 /plan 产出.

用法 (nav2_nav 干跑运行中, with_forwarder:=false 即可):
  python3 scripts/verify_f4_planning.py --once
  python3 scripts/verify_f4_planning.py --once --distance 2.0 --timeout 30

通过标准 (E2E F4):
  当前位姿前方 distance m 发 /goal_pose 后 timeout 内收到 /plan 且 poses>=2
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def sample_odom(node: Node, timeout: float = 5.0) -> Odometry | None:
    holder: list[Odometry] = []

    def cb(m: Odometry) -> None:
        holder.append(m)

    node.create_subscription(Odometry, "/lio/odom", cb, ODOM_QOS)
    t0 = time.time()
    while not holder and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    return holder[0] if holder else None


def main() -> int:
    parser = argparse.ArgumentParser(description="F4 规划可行验收 (Nav2)")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--distance", type=float, default=2.0, help="goal 距当前 xy (m)")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--min-poses", type=int, default=2, help="plan poses 最少")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--plan-topic", default="/plan")
    args = parser.parse_args()

    rclpy.init()
    node = Node("verify_f4_planning")

    odom = sample_odom(node)
    if not odom:
        print("ERROR: 无 /lio/odom", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    ox = odom.pose.pose.position.x
    oy = odom.pose.pose.position.y
    oz = odom.pose.pose.position.z
    q = odom.pose.pose.orientation
    yaw = quat_yaw(q.x, q.y, q.z, q.w)

    gx = ox + args.distance * math.cos(yaw)
    gy = oy + args.distance * math.sin(yaw)
    gz = max(oz, 0.0)

    received: list[Path] = []

    def on_plan(msg: Path) -> None:
        received.append(msg)

    node.create_subscription(Path, args.plan_topic, on_plan, 10)
    pub = node.create_publisher(PoseStamped, args.goal_topic, 10)

    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.position.z = gz
    goal.pose.orientation.w = q.w
    goal.pose.orientation.x = q.x
    goal.pose.orientation.y = q.y
    goal.pose.orientation.z = q.z

    print(f"F4: 当前 ({ox:.2f}, {oy:.2f}) yaw={math.degrees(yaw):.1f}°")
    print(f"    发 goal ({gx:.2f}, {gy:.2f}, {gz:.2f}) 距离 {args.distance} m")

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)

    pub.publish(goal)
    t0 = time.time()
    while not received and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not received:
        print(f"FAIL: {args.timeout}s 内无 {args.plan_topic}", file=sys.stderr)
        print("  提示: 确认 global_map_to_grid 已写 /dev/shm/motionslam_map", file=sys.stderr)
        return 1

    msg = received[0]
    n_poses = len(msg.poses)
    print(f"  收到 plan frame={msg.header.frame_id} poses={n_poses}")
    if n_poses >= args.min_poses:
        print("OK: F4 规划可行通过")
        return 0
    print(f"FAIL: poses={n_poses} < {args.min_poses}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
