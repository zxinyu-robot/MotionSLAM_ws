#!/usr/bin/env python3
"""估计 /lio/cloud_world 相对 /map_cloud 需要的 yaw 修正 (度).

输出 dyaw: 把 live scan 绕机器人中心旋转 dyaw 后与 map 重合最好。
定位 yaw 应约 imu_yaw - dyaw (与 tune_reloc_yaw --delta-yaw-deg 符号一致).

  python3 /ws/scripts/estimate_scan_map_yaw.py
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from lio_pose_utils import yaw_deg_from_quat


class N(Node):
    def __init__(self) -> None:
        super().__init__("estimate_scan_map_yaw")
        self.map = self.live = self.odom = None
        self.create_subscription(PointCloud2, "/map_cloud", lambda m: setattr(self, "map", m), 1)
        self.create_subscription(PointCloud2, "/lio/cloud_world", lambda m: setattr(self, "live", m), 1)
        self.create_subscription(Odometry, "/lio/odom", lambda m: setattr(self, "odom", m), 10)


def score(map_keys, live_xy, cx, cy, dyaw, leaf=0.15):
    c, s = math.cos(dyaw), math.sin(dyaw)
    n = 0
    for x, y in live_xy:
        lx, ly = x - cx, y - cy
        wx, wy = c * lx - s * ly + cx, s * lx + c * ly + cy
        if (int(wx / leaf), int(wy / leaf)) in map_keys:
            n += 1
    return n


def main() -> int:
    rclpy.init()
    n = N()
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.map and n.live and n.odom:
            break
    if not (n.map and n.live and n.odom):
        print("ERROR: 需要 /map_cloud /lio/cloud_world /lio/odom", file=sys.stderr)
        return 1

    import numpy as np

    map_pts = np.array([(p[0], p[1]) for p in pc2.read_points(n.map, ("x", "y", "z"), skip_nans=True)])
    live_pts = np.array([(p[0], p[1]) for p in pc2.read_points(n.live, ("x", "y", "z"), skip_nans=True)])
    cx = n.odom.pose.pose.position.x
    cy = n.odom.pose.pose.position.y
    imu_yaw = yaw_deg_from_quat(n.odom.pose.pose.orientation)

    map_xy = map_pts[np.hypot(map_pts[:, 0] - cx, map_pts[:, 1] - cy) < 10][:, :2]
    live_xy = live_pts[np.hypot(live_pts[:, 0] - cx, live_pts[:, 1] - cy) < 5][:, :2]
    live_xy = live_xy[:: max(1, len(live_xy) // 8000)]
    keys = set((int(x / 0.15), int(y / 0.15)) for x, y in map_xy)

    best_d, best_s = 0, 0
    for d in range(-45, 46, 1):
        sc = score(keys, live_xy, cx, cy, math.radians(d))
        if sc > best_s:
            best_s, best_d = sc, d
    for d in [best_d + x / 10.0 for x in range(-30, 31)]:
        sc = score(keys, live_xy, cx, cy, math.radians(d))
        if sc > best_s:
            best_s, best_d = sc, d

    print(f"pos=({cx:.2f},{cy:.2f}) imu_yaw={imu_yaw:.1f}°")
    print(f"建议 dyaw={best_d:.1f}° (score={best_s})")
    print(f"若 Foxglove 目视仍偏, 可: python3 tune_reloc_yaw.py --delta-yaw-deg {-best_d:.1f} --write")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
