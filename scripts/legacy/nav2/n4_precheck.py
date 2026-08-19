#!/usr/bin/env python3
"""N4 发 goal 前复检: robot/goal cost + 2m 规划 span."""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def cost_at(cm: OccupancyGrid, x: float, y: float) -> int | None:
    o = cm.info.origin.position
    res = cm.info.resolution
    mx = int((x - o.x) / res)
    my = int((y - o.y) / res)
    if mx < 0 or my < 0 or mx >= cm.info.width or my >= cm.info.height:
        return None
    return int(cm.data[my * cm.info.width + mx])


class Precheck(Node):
    def __init__(self) -> None:
        super().__init__("n4_precheck")
        self.odom: Odometry | None = None
        self.cm: OccupancyGrid | None = None
        self.create_subscription(Odometry, "/lio/robo/odom", self._on_odom, 10)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_cm, 10
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_cm, MAP_QOS
        )
        self.plan = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_cm(self, msg: OccupancyGrid) -> None:
        self.cm = msg


def main() -> int:
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    rclpy.init()
    node = Precheck()
    t0 = time.time()
    while (node.odom is None or node.cm is None) and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None or node.cm is None:
        print("ERROR: no odom/costmap")
        return 1

    p = node.odom.pose.pose.position
    y = yaw(node.odom.pose.pose.orientation)
    rc = cost_at(node.cm, p.x, p.y)
    gx = p.x + distance * math.cos(y)
    gy = p.y + distance * math.sin(y)
    gc = cost_at(node.cm, gx, gy)
    robot_free = rc is not None and rc <= 1
    goal_free = gc is not None and gc <= 1

    print(f"=== N4 复检 (goal +{distance:.0f}m) ===")
    print(
        f"robot=({p.x:.2f},{p.y:.2f}) yaw={math.degrees(y):.1f}° "
        f"cost={rc} ({'OK' if robot_free else 'BAD'})"
    )
    print(
        f"goal(+{distance:.0f}m)=({gx:.2f},{gy:.2f}) cost={gc} "
        f"({'OK' if goal_free else 'BAD'})"
    )

    plan_ok = False
    if node.plan.wait_for_server(timeout_sec=5.0):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation = node.odom.pose.pose.orientation
        req = ComputePathToPose.Goal()
        req.goal = goal
        req.planner_id = "GridBased"
        fut = node.plan.send_goal_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        handle = fut.result() if fut.done() else None
        if handle and handle.accepted:
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(node, res_fut, timeout_sec=15.0)
            if res_fut.done():
                path = res_fut.result().result.path
                if len(path.poses) >= 2:
                    s = path.poses[0].pose.position
                    t = path.poses[-1].pose.position
                    span = math.hypot(t.x - s.x, t.y - s.y)
                    end = math.hypot(t.x - p.x, t.y - p.y)
                    plan_ok = span >= 1.5
                    print(
                        f"plan poses={len(path.poses)} span={span:.2f}m end={end:.2f}m"
                    )

    ready = robot_free and goal_free and plan_ok
    print(f"N4_READY={'YES' if ready else 'NO'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())
