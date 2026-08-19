#!/usr/bin/env python3
"""对照现场方位: 打印 robo 朝向与 map 前后方点密度 (仅参考).

NOTE: 玻璃/工位角点密度不可靠; **以 Foxglove 调头后 /map_cloud 与 /lio/cloud_world 目视一致为准**。
init_pose 的 yaw 请用 read_reloc_init_pose.py (/lio/odom), 勿填 robo yaw。
"""
from __future__ import annotations

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from lio_pose_utils import init_pose_line, quat_yaw_rad, yaw_deg_from_quat


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("verify_scene_heading")
        self.odom: Odometry | None = None
        self.imu_odom: Odometry | None = None
        self.cloud: PointCloud2 | None = None
        self.create_subscription(Odometry, "/lio/robo/odom", self._on_odom, 10)
        self.create_subscription(Odometry, "/lio/odom", self._on_imu, 10)
        self.create_subscription(PointCloud2, "/map_cloud", self._on_cloud, 1)

    def _on_imu(self, msg: Odometry) -> None:
        self.imu_odom = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.cloud = msg


def sector_count(
    pts: np.ndarray, px: float, py: float, yaw: float, fwd_x: float, fwd_y: float
) -> int:
    cnt = 0
    for x, y, z in pts:
        lx = (x - px) * fwd_x + (y - py) * fwd_y
        ly = -(x - px) * fwd_y + (y - py) * fwd_x
        if 2.0 < lx < 5.0 and abs(ly) < 1.5:
            cnt += 1
    return cnt


def main() -> int:
    rclpy.init()
    node = Probe()
    deadline = time.time() + 15.0
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.odom and node.cloud and node.imu_odom:
            break

    if not node.odom or not node.cloud:
        print("ERROR: 需要 /lio/robo/odom 与 /map_cloud", file=sys.stderr)
        return 1

    o = node.odom.pose.pose
    yaw = quat_yaw_rad(o.orientation)
    fx, fy = math.cos(yaw), math.sin(yaw)
    pts = []
    for p in pc2.read_points(node.cloud, ("x", "y", "z"), skip_nans=True):
        pts.append((float(p[0]), float(p[1]), float(p[2])))
    arr = np.asarray(pts)
    px, py = o.position.x, o.position.y

    fwd = sector_count(arr, px, py, yaw, fx, fy)
    back = sector_count(arr, px, py, yaw, -fx, -fy)
    left = sector_count(arr, px, py, yaw, -fy, fx)
    right = sector_count(arr, px, py, yaw, fy, -fx)

    print(f"robot xy=({px:.2f},{py:.2f}) robo_yaw_deg={math.degrees(yaw):.1f}")
    if node.imu_odom:
        imu = node.imu_odom.pose.pose
        iy = yaw_deg_from_quat(imu.orientation)
        print(
            f"ICP init_pose yaw_deg={iy:.1f}  (from /lio/odom; xy=({imu.position.x:.2f},{imu.position.y:.2f}))"
        )
        print(f"  yaml 行: {init_pose_line(imu.position.x, imu.position.y, imu.position.z, iy)}")
    print(f"map_cloud 2–5m:  +X_fwd(glass?)={fwd}  -X_back(cabinet?)={back}")
    print(f"                 +Y_left(工位?)={left}  -Y_right(工位?)={right}")

    ok = back >= fwd * 0.85
    if ok:
        print("NOTE(密度): 后方点数 ≥ 前方 (墙柜可能更密)")
    else:
        print("NOTE(密度): 前方点数 > 后方 — 角点/玻璃反射常见, 请用 Foxglove 目视判定")
    print("认位: 狗头=橙色 ARROW; 目视一致后: python3 /ws/scripts/read_reloc_init_pose.py")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
