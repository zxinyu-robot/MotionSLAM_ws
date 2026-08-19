#!/usr/bin/env python3
"""证据 bag 快速诊断: cmd_vel vs odom 是否脱节."""
from __future__ import annotations

import argparse
import math
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag_path, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )

    vx_list: list[float] = []
    odom_xy: list[tuple[float, float]] = []

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == "/cmd_vel":
            m = deserialize_message(data, Twist)
            vx_list.append(m.linear.x)
        elif topic == "/lio/robo/odom":
            m = deserialize_message(data, Odometry)
            odom_xy.append((m.pose.pose.position.x, m.pose.pose.position.y))

    if not vx_list or not odom_xy:
        print("FAIL: bag 缺 cmd_vel 或 odom")
        return 1

    x0, y0 = odom_xy[0]
    max_travel = max(math.hypot(x - x0, y - y0) for x, y in odom_xy)
    nz_vx = sum(1 for v in vx_list if abs(v) > 0.05)
    max_vx = max(vx_list)
    mean_vx = sum(vx_list) / len(vx_list)

    print(f"cmd_vel: n={len(vx_list)} max_vx={max_vx:.3f} mean_vx={mean_vx:.3f} |vx|>0.05: {nz_vx}")
    print(f"odom: n={len(odom_xy)} start=({x0:.3f},{y0:.3f}) end=({odom_xy[-1][0]:.3f},{odom_xy[-1][1]:.3f})")
    print(f"max_travel={max_travel:.3f} m")

    if nz_vx > 100 and max_travel < 0.3:
        print("DIAG: Nav2 有速度指令但机身几乎不动 → 查 forwarder/obstacles_avoid/Sport 抢占")
    elif max_travel >= 0.3:
        print("DIAG: 机身有位移, 未到达更可能是规划/目标问题")
    else:
        print("DIAG: 指令弱或局部震荡, 查 MPPI/costmap")
    return 0


if __name__ == "__main__":
    sys.exit(main())
