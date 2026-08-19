#!/usr/bin/env python3
"""H5 到达验收共用工具 (Nav2 / SCAN 脚本共享，无 nav2_msgs 依赖)."""
from __future__ import annotations

import math
import subprocess

from geometry_msgs.msg import Quaternion
from motionslam_msgs.msg import MotionCommand
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def quat_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return quat_rpy(x, y, z, w)[2]


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def dist2(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def zero_motion_command(frame_id: str = "body") -> MotionCommand:
    msg = MotionCommand()
    msg.header.frame_id = frame_id
    msg.vx = 0.0
    msg.vy = 0.0
    msg.yaw_rate = 0.0
    msg.traj_id = 0
    msg.sequence = 0
    return msg


def run_estop_cpp() -> None:
    """调用 C++ estop_go2 (Sport StopMove + 零速)。"""
    try:
        subprocess.run(
            ["ros2", "run", "motionslam_pipeline", "estop_go2"],
            check=False,
            timeout=8,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass
