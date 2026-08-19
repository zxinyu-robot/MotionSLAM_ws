#!/usr/bin/env python3
"""SCAN-Planner B-spline -> Foxglove 友好规划轨迹.

订阅 scan_planner 每次 replan 发布的 /planning/bspline, 采样为:
  /planning/trajectory_path   nav_msgs/Path          (3D 面板 Path, 推荐)
  /planning/trajectory_marker visualization_msgs/Marker LINE_STRIP

C++ 侧 /optimal_list 为 Marker 且仅在 replan 时更新; 本节点便于 Foxglove 直接看实时规划曲线。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import Bspline
from visualization_msgs.msg import Marker

def _deboor(t: float, order: int, knots: list[float], ctrl: list[tuple[float, float, float]]):
    k = order
    n = len(ctrl)
    if n <= k or len(knots) < n + k + 1:
        return None
    if t <= knots[k]:
        return ctrl[0]
    if t >= knots[n]:
        return ctrl[-1]
    span = k
    while span < n - 1 and t >= knots[span + 1]:
        span += 1
    d = [list(ctrl[span - k + i]) for i in range(k + 1)]
    for r in range(1, k + 1):
        for j in range(k, r - 1, -1):
            i = span - k + j
            denom = knots[i + k + 1 - r] - knots[i]
            alpha = 0.0 if abs(denom) < 1e-12 else (t - knots[i]) / denom
            d[j] = [
                (1.0 - alpha) * d[j - 1][c] + alpha * d[j][c]
                for c in range(3)
            ]
    return tuple(d[k])


def _sample_bspline(msg: Bspline, dt: float) -> list[tuple[float, float, float]]:
    if len(msg.pos_pts) < 2:
        return [(p.x, p.y, p.z) for p in msg.pos_pts]

    ctrl = [(p.x, p.y, p.z) for p in msg.pos_pts]
    knots = list(msg.knots)
    k = int(msg.order)
    if len(knots) < len(ctrl) + k + 1:
        return ctrl

    t0 = knots[k]
    t1 = knots[len(ctrl)]
    if t1 <= t0 + 1e-6:
        return ctrl

    n = max(2, int(math.ceil((t1 - t0) / max(dt, 0.02))) + 1)
    out: list[tuple[float, float, float]] = []
    for i in range(n):
        t = t0 + (t1 - t0) * i / (n - 1)
        pt = _deboor(t, k, knots, ctrl)
        if pt is not None:
            out.append(pt)
    return out or ctrl


class PlanningTrajectoryViz(Node):
    def __init__(self) -> None:
        super().__init__("planning_trajectory_viz")
        self.declare_parameter("bspline_topic", "/planning/bspline")
        self.declare_parameter("path_topic", "/planning/trajectory_path")
        self.declare_parameter("marker_topic", "/planning/trajectory_marker")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("sample_dt", 0.08)
        self.declare_parameter("line_width", 0.06)
        self.declare_parameter("line_color_r", 1.0)
        self.declare_parameter("line_color_g", 0.45)
        self.declare_parameter("line_color_b", 0.0)

        bspline_topic = self.get_parameter("bspline_topic").value
        path_topic = self.get_parameter("path_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._sample_dt = float(self.get_parameter("sample_dt").value)
        self._line_width = float(self.get_parameter("line_width").value)
        self._color = (
            float(self.get_parameter("line_color_r").value),
            float(self.get_parameter("line_color_g").value),
            float(self.get_parameter("line_color_b").value),
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._path_pub = self.create_publisher(Path, path_topic, qos)
        self._marker_pub = self.create_publisher(Marker, marker_topic, qos)
        self.create_subscription(Bspline, bspline_topic, self._on_bspline, 10)
        self.get_logger().info(
            f"规划轨迹 viz: {bspline_topic} -> {path_topic}, {marker_topic}"
        )

    def _on_bspline(self, msg: Bspline) -> None:
        if not msg.pos_pts:
            return
        frame = self._frame_id or "world"
        stamp = self.get_clock().now().to_msg()
        samples = _sample_bspline(msg, self._sample_dt)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame
        for x, y, z in samples:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = "scan_planner_traj"
        marker.id = int(msg.traj_id) & 0x7FFFFFFF
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._line_width
        marker.color.r = self._color[0]
        marker.color.g = self._color[1]
        marker.color.b = self._color[2]
        marker.color.a = 0.95
        for x, y, z in samples:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = z
            marker.points.append(pt)
        self._marker_pub.publish(marker)

        self.get_logger().info(
            f"traj_id={msg.traj_id} samples={len(samples)} duration_pts={len(msg.pos_pts)}"
        )


def main() -> None:
    rclpy.init()
    node = PlanningTrajectoryViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
