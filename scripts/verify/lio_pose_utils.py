"""LIO 位姿与重定位 init_pose 的 yaw 约定 (供 scripts 共用).

init_pose / ICP 初值 yaw 使用 **/lio/odom** (与 /lio/robo/odom 的 yaw 在 odom_robo.yaw=0 时一致).
odom_robo 的 **pitch** (Go2 重建图 yaml≈11.8°) 用于拉平机身俯仰, 不是把地图「转斜」.
"""
from __future__ import annotations

import math
from typing import Tuple

from geometry_msgs.msg import Quaternion


def quat_yaw_rad(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_yaw_deg(yaw_deg: float) -> float:
    while yaw_deg > 180.0:
        yaw_deg -= 360.0
    while yaw_deg <= -180.0:
        yaw_deg += 360.0
    return yaw_deg


def robo_to_init_pose_yaw_deg(robo_yaw_deg: float, odom_robo_yaw_deg: float = 0.0) -> float:
    """init_pose 第 6 项 (度): 由机身 yaw + yaml 中 odom_robo.yaw 得到 ICP 系 yaw."""
    return normalize_yaw_deg(robo_yaw_deg + odom_robo_yaw_deg)


def init_pose_line(
    x: float,
    y: float,
    z: float,
    imu_yaw_deg: float,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
) -> str:
    return f"[{x:.2f}, {y:.2f}, {z:.2f}, {roll_deg:.1f}, {pitch_deg:.1f}, {imu_yaw_deg:.1f}]"


def yaw_deg_from_quat(q: Quaternion) -> float:
    return math.degrees(quat_yaw_rad(q))
