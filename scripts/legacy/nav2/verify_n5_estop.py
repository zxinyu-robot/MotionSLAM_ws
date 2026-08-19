#!/usr/bin/env python3
"""N5: 运行中 estop 后狗是否静止."""
from __future__ import annotations

import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def dist2(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def run_estop() -> float:
    t0 = time.time()
    subprocess.run(
        ["ros2", "run", "motionslam_pipeline", "estop_go2"],
        check=False,
        timeout=8,
    )
    return time.time() - t0


class N5Node(Node):
    def __init__(self) -> None:
        super().__init__("verify_n5_estop")
        self.odom: Odometry | None = None
        self.create_subscription(Odometry, "/lio/robo/odom", self._cb, ODOM_QOS)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _cb(self, msg: Odometry) -> None:
        self.odom = msg

    def wait_odom(self, timeout: float = 15.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.odom is not None:
                return True
        return False

    def send_goal(self, distance: float = 2.0) -> None:
        assert self.odom is not None
        p = self.odom.pose.pose.position
        yaw = quat_yaw(
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        )
        gx = p.x + distance * math.cos(yaw)
        gy = p.y + distance * math.sin(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "world"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation = yaw_to_quat(yaw)
        self.client.wait_for_server(timeout_sec=20.0)
        self.client.send_goal_async(goal)
        print(f"  goal ({gx:.2f}, {gy:.2f}) 距离 {distance:.1f} m")


def sample_motion(node: N5Node, seconds: float) -> tuple[float, float, float]:
    assert node.odom is not None
    x0 = node.odom.pose.pose.position.x
    y0 = node.odom.pose.pose.position.y
    t0 = time.time()
    max_step = 0.0
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None:
            continue
        step = dist2(x0, y0, node.odom.pose.pose.position.x, node.odom.pose.pose.position.y)
        max_step = max(max_step, step)
    assert node.odom is not None
    vx = node.odom.twist.twist.linear.x
    vy = node.odom.twist.twist.linear.y
    speed = math.hypot(vx, vy)
    return max_step, speed, time.time() - t0


def main() -> int:
    rclpy.init()
    node = N5Node()
    print("N5: 等待 /lio/robo/odom ...")
    if not node.wait_odom():
        print("FAIL: 无 odom")
        rclpy.shutdown()
        return 1
    assert node.odom is not None
    p = node.odom.pose.pose.position
    print(f"  起点 ({p.x:.2f}, {p.y:.2f})")
    print("  发送 NavigateToPose, 3s 后 estop_go2 ...")
    node.send_goal(2.0)
    time.sleep(3.0)
    pre_step, pre_speed, _ = sample_motion(node, 0.5)
    print(f"  estop 前 0.5s: 位移 {pre_step:.3f} m, 速度 {pre_speed:.3f} m/s")
    estop_latency = run_estop()
    print(f"  estop_go2 调用耗时 {estop_latency:.2f} s")
    post_step, post_speed, post_dt = sample_motion(node, 3.0)
    print(f"  estop 后 3s: 位移 {post_step:.3f} m, 末速度 {post_speed:.3f} m/s")
    still = post_step > 0.05 or post_speed > 0.05
    if still:
        print(f"FAIL: estop 后仍运动 (位移 {post_step:.3f} m, 速度 {post_speed:.3f} m/s)")
        rclpy.shutdown()
        return 1
    print("PASS: estop 后 3s 内静止")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
