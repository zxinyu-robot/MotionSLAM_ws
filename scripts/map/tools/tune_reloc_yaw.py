#!/usr/bin/env python3
"""按 Foxglove 目视夹角微调 reloc_spots 的 yaw, 然后重启 relocation 做 ICP.

夹角约定 (俯视, world 系):
  - 当前 scan 相对 map **逆时针** 转多了 → 定位 yaw 偏大 → --delta-yaw-deg 为 **负**
  - scan 相对 map **顺时针** 转多了 → --delta-yaw-deg 为 **正**

用法 (栈在跑, 狗已停稳):
  python3 /ws/scripts/tune_reloc_yaw.py --spot glass_facing --delta-yaw-deg -8 --write
  # 然后重启重定位 (见脚本末尾提示)

仅预览不写文件:
  python3 /ws/scripts/tune_reloc_yaw.py --delta-yaw-deg 5
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lio_pose_utils import normalize_yaw_deg, yaw_deg_from_quat

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


class Sample(Node):
    def __init__(self) -> None:
        super().__init__("tune_reloc_yaw")
        self.imu: Odometry | None = None
        self.create_subscription(Odometry, "/lio/odom", lambda m: setattr(self, "imu", m), ODOM_QOS)


def main() -> int:
    parser = argparse.ArgumentParser(description="微调 reloc_spot yaw")
    parser.add_argument("--spot", default="glass_facing")
    parser.add_argument("--delta-yaw-deg", type=float, required=True)
    parser.add_argument("--spots-file", default=DEFAULT_SPOTS)
    parser.add_argument("--write", action="store_true", help="写回 reloc_spots.yaml")
    args = parser.parse_args()

    rclpy.init()
    node = Sample()
    t0 = time.time()
    while node.imu is None and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.imu is None:
        print("ERROR: 无 /lio/odom", file=sys.stderr)
        return 1

    imu = node.imu.pose.pose
    yaw_now = yaw_deg_from_quat(imu.orientation)
    yaw_new = normalize_yaw_deg(yaw_now + args.delta_yaw_deg)

    if not os.path.isfile(args.spots_file):
        print(f"ERROR: 找不到 {args.spots_file}", file=sys.stderr)
        return 1

    with open(args.spots_file, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    spots = data.get("spots") or {}
    if args.spot not in spots:
        print(f"ERROR: spot 不存在: {args.spot}", file=sys.stderr)
        return 1

    spot = spots[args.spot]
    old_yaw = float(spot.get("yaw_deg", 0.0))
    spot["x"] = round(float(imu.position.x), 2)
    spot["y"] = round(float(imu.position.y), 2)
    spot["z"] = round(float(imu.position.z), 2)
    spot["yaw_deg"] = round(yaw_new, 1)
    spot["description"] = spot.get("description", "") + f" tune delta={args.delta_yaw_deg:+.1f}"

    print(f"spot={args.spot}")
    print(f"  当前 /lio/odom yaw={yaw_now:.1f}°")
    print(f"  delta={args.delta_yaw_deg:+.1f}° → 新 init_pose yaw={yaw_new:.1f}° (spot 内 {old_yaw:.1f}→{spot['yaw_deg']:.1f})")
    print(f"  xy=({spot['x']}, {spot['y']})")

    if args.write:
        with open(args.spots_file, "w", encoding="utf-8") as f:
            yaml.dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
        print(f"已写入 {args.spots_file}")
        print(
            "\n请重启重定位使 ICP 用新初值 (Hybrid 分支):\n"
            "  /ws/scripts/nav/stop_nav.sh\n"
            "  git checkout feature/hybrid-baseline\n"
            "  ./scripts/nav/start_hybrid_nav.sh "
            f"reloc_spot:={args.spot}\n"
            "叠图: 只看 3D 中 /lio/cloud_world + /map_cloud (Fixed Frame=world), "
            "勿把 Map 面板 /map 叠进 3D 误判夹角。"
        )
    else:
        print("预览模式 (加 --write 写 yaml)")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
