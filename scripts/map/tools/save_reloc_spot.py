#!/usr/bin/env python3
"""认位后把当前 /lio/odom 位姿写入 reloc_spots.yaml (免改 super_lio_reloc_mid360.yaml).

用法 (栈在跑、Foxglove 叠图 OK):
  python3 /ws/scripts/save_reloc_spot.py --name glass_facing
  python3 /ws/scripts/save_reloc_spot.py --name glass_facing --set-default
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from lio_pose_utils import init_pose_line, yaw_deg_from_quat

ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

DEFAULT_SPOTS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src",
    "motionslam_bringup",
    "config",
    "reloc_spots.yaml",
)
# 容器内工作区
CONTAINER_SPOTS = "/ws/src/motionslam_bringup/config/reloc_spots.yaml"


class Sample(Node):
    def __init__(self) -> None:
        super().__init__("save_reloc_spot")
        self.imu: Odometry | None = None
        self.create_subscription(Odometry, "/lio/odom", lambda m: setattr(self, "imu", m), ODOM_QOS)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True, help="工位名 (字母数字下划线)")
    parser.add_argument("--description", default="", help="说明")
    parser.add_argument("--set-default", action="store_true", help="设为 default_spot")
    parser.add_argument("--spots-file", default="", help="默认仓库 config/reloc_spots.yaml")
    parser.add_argument("--samples", type=int, default=3)
    args = parser.parse_args()

    spots_path = args.spots_file or (
        CONTAINER_SPOTS if os.path.isfile(CONTAINER_SPOTS) else DEFAULT_SPOTS
    )

    rclpy.init()
    node = Sample()
    xs: list[float] = []
    deadline = time.time() + 20.0
    while rclpy.ok() and time.time() < deadline and len(xs) < args.samples:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.imu:
            p = node.imu.pose.pose.position
            y = yaw_deg_from_quat(node.imu.pose.pose.orientation)
            xs.append((p.x, p.y, p.z, y))
            time.sleep(0.25)

    if not xs:
        print("ERROR: 无 /lio/odom", file=sys.stderr)
        return 1

    ax = sum(t[0] for t in xs) / len(xs)
    ay = sum(t[1] for t in xs) / len(xs)
    az = sum(t[2] for t in xs) / len(xs)
    ayaw = sum(t[3] for t in xs) / len(xs)

    with open(spots_path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("spots", {})
    data["spots"][args.name] = {
        "x": round(ax, 2),
        "y": round(ay, 2),
        "z": round(az, 2),
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": round(ayaw, 1),
        "description": args.description or f"saved {time.strftime('%Y-%m-%d')}",
    }
    if args.set_default:
        data["default_spot"] = args.name

    with open(spots_path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

    print(f"已写入 {spots_path} spot={args.name}")
    print(f"  init_pose {init_pose_line(ax, ay, az, ayaw)}")
    print(f"启动 (Hybrid): git checkout feature/hybrid-baseline && ./scripts/nav/start_hybrid_nav.sh reloc_spot:={args.name}")
    print(f"启动 (Demo):     ./scripts/motionslam nav start")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
