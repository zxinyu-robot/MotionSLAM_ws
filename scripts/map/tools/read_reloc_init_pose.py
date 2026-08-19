#!/usr/bin/env python3
"""读取当前位姿并输出正确的 lio.relocation.init_pose (尤其 yaw 用 /lio/odom).

用法 (导航/reloc 已运行):
  python3 /ws/scripts/read_reloc_init_pose.py
  python3 /ws/scripts/read_reloc_init_pose.py --samples 5
  python3 /ws/scripts/read_reloc_init_pose.py --odom-robo-yaw-deg 180

NOTE: init_pose yaw 必须对齐 /lio/odom; 勿填 /lio/robo/odom 的 yaw (odom_robo.yaw≠0 时会差 180°).
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from lio_pose_utils import (
    init_pose_line,
    normalize_yaw_deg,
    robo_to_init_pose_yaw_deg,
    yaw_deg_from_quat,
)

DEFAULT_RELOC_YAML = "/ws/src/motionslam_bringup/config/super_lio_reloc_mid360.yaml"


def load_odom_robo_yaw_deg(yaml_path: str) -> float:
    """读 lio.extrinsic.odom_robo 第 6 项 yaw (度); 当前 Go2 重建图多为 pitch=11.8, yaw=0."""
    import re

    path = yaml_path
    if not os.path.isfile(path):
        alt = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "src",
            "motionslam_bringup",
            "config",
            "super_lio_reloc_mid360.yaml",
        )
        path = alt if os.path.isfile(alt) else path
    if not os.path.isfile(path):
        return 0.0
    text = open(path, encoding="utf-8").read()
    m = re.search(
        r"lio\.extrinsic\.odom_robo:\s*\[([^\]]+)\]", text, re.MULTILINE
    )
    if not m:
        return 0.0
    parts = [p.strip() for p in m.group(1).split(",")]
    if len(parts) < 6:
        return 0.0
    return float(parts[5])

ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class Reader(Node):
    def __init__(self) -> None:
        super().__init__("read_reloc_init_pose")
        self.imu: Odometry | None = None
        self.robo: Odometry | None = None
        self.create_subscription(Odometry, "/lio/odom", self._imu, ODOM_QOS)
        self.create_subscription(Odometry, "/lio/robo/odom", self._robo, ODOM_QOS)

    def _imu(self, msg: Odometry) -> None:
        self.imu = msg

    def _robo(self, msg: Odometry) -> None:
        self.robo = msg


def main() -> int:
    parser = argparse.ArgumentParser(description="输出 reloc init_pose (ICP 系 yaw)")
    parser.add_argument("--samples", type=int, default=3, help="采样次数取平均 yaw")
    parser.add_argument(
        "--odom-robo-yaw-deg",
        type=float,
        default=None,
        help="odom_robo 的 yaw 分量 (度); 默认从 super_lio_reloc_mid360.yaml 读取",
    )
    parser.add_argument(
        "--reloc-yaml",
        default=DEFAULT_RELOC_YAML,
        help="含 lio.extrinsic.odom_robo 的配置路径",
    )
    args = parser.parse_args()
    odom_robo_yaw = (
        args.odom_robo_yaw_deg
        if args.odom_robo_yaw_deg is not None
        else load_odom_robo_yaw_deg(args.reloc_yaml)
    )

    rclpy.init()
    node = Reader()
    imu_yaws: list[float] = []
    robo_yaws: list[float] = []
    last_imu = last_robo = None

    deadline = time.time() + 20.0
    while rclpy.ok() and time.time() < deadline and len(imu_yaws) < args.samples:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.imu and node.robo:
            last_imu, last_robo = node.imu, node.robo
            imu_yaws.append(yaw_deg_from_quat(node.imu.pose.pose.orientation))
            robo_yaws.append(yaw_deg_from_quat(node.robo.pose.pose.orientation))
            time.sleep(0.3)

    if not last_imu or not last_robo:
        print("ERROR: 需要 /lio/odom 与 /lio/robo/odom", file=sys.stderr)
        return 1

    imu_yaw = sum(imu_yaws) / len(imu_yaws)
    robo_yaw = sum(robo_yaws) / len(robo_yaws)
    derived = robo_to_init_pose_yaw_deg(robo_yaw, odom_robo_yaw)
    diff = normalize_yaw_deg(derived - imu_yaw)

    ip = last_imu.pose.pose.position
    rp = last_robo.pose.pose.position
    line = init_pose_line(ip.x, ip.y, ip.z, imu_yaw)

    print(f"/lio/robo/odom  狗头 yaw_deg={robo_yaw:.2f}  xy=({rp.x:.2f},{rp.y:.2f})")
    print(f"/lio/odom       ICP  yaw_deg={imu_yaw:.2f}  xy=({ip.x:.2f},{ip.y:.2f})")
    print(
        f"由 robo+odom_robo.yaw({odom_robo_yaw:.1f}°) 推算 ICP yaw={derived:.2f}  "
        f"(与 /lio/odom 差 {diff:.2f}°)"
    )
    print(
        "NOTE: odom_robo 的 pitch(当前 yaml≈11.8°) 只影响俯仰, "
        "不改变 world yaw; 狗头/Nav2 看 /lio/robo/odom + TF(level_orientation)"
    )
    if abs(diff) > 5.0:
        print("WARN: 推算与 /lio/odom 差 >5° — 请核对 yaml 中 odom_robo.yaw", file=sys.stderr)
    print(f"\nlio.relocation.init_pose: {line}")
    print("  ↑ 第6项必须用 ICP/odom yaw; Foxglove 认位后运行本脚本再改 yaml 并重启 reloc")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
