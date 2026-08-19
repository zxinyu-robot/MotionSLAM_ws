#!/usr/bin/env python3
"""Nav2 障碍/规划/控制只读诊断 (方案 3).

检查:
  - /lio/cloud_world 机器人 3 m 内疑似障碍点数
  - /local_costmap/costmap 与 /global_costmap/costmap 致命/高代价栅格
  - /plan 长度与相对起终点连线的最大侧偏
  - /cmd_vel 与 /cmd_vel_nav 是否 Nav2 有输出、forwarder 是否收到
  - 近期 controller 日志关键词 (需容器内读 log 文件)

用法:
  python3 scripts/diag_nav_obstacle.py
  python3 scripts/diag_nav_obstacle.py --navigate-distance 3 --monitor-sec 60
"""
from __future__ import annotations

import argparse
import math
import os
import re
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def lateral_max(sx: float, sy: float, gx: float, gy: float, path: Path) -> float:
    if not path.poses:
        return 0.0
    dx, dy = gx - sx, gy - sy
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return 0.0
    nx, ny = -dy / L, dx / L
    m = 0.0
    for p in path.poses:
        px, py = p.pose.position.x, p.pose.position.y
        m = max(m, abs((px - sx) * nx + (py - sy) * ny))
    return m


def count_cloud_near_robot(msg: PointCloud2, rx: float, ry: float, rz: float, radius: float,
                           z_min: float, z_max: float) -> Tuple[int, int]:
    """返回 (半径内总点数, 高度过滤后障碍点数)."""
    total = 0
    obs = 0
    r2 = radius * radius
    ox = oy = oz = 0
    for f in msg.fields:
        if f.name == "x":
            ox = f.offset
        elif f.name == "y":
            oy = f.offset
        elif f.name == "z":
            oz = f.offset
    if oz == 0 and not any(f.name == "z" for f in msg.fields):
        return 0, 0
    step = msg.point_step
    data = msg.data
    n = len(data) // step
    for i in range(n):
        base = i * step
        x = struct.unpack_from("f", data, base + ox)[0]
        y = struct.unpack_from("f", data, base + oy)[0]
        z = struct.unpack_from("f", data, base + oz)[0]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        dx, dy = x - rx, y - ry
        if dx * dx + dy * dy > r2:
            continue
        total += 1
        h = z - rz
        if z_min <= h <= z_max:
            obs += 1
    return total, obs


def costmap_stats(grid: OccupancyGrid, wx: float, wy: float, radius_m: float) -> dict:
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    lethal_all = sum(1 for c in grid.data if c >= 99)
    high_all = sum(1 for c in grid.data if 50 <= c < 99)
    cx = int((wx - ox) / res)
    cy = int((wy - oy) / res)
    r_cells = int(radius_m / res) + 1
    lethal_near = high_near = unknown_near = 0
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dx * dx + dy * dy > r_cells * r_cells:
                continue
            gx, gy = cx + dx, cy + dy
            if gx < 0 or gy < 0 or gx >= w or gy >= h:
                continue
            v = grid.data[gy * w + gx]
            if v < 0:
                unknown_near += 1
            elif v >= 99:
                lethal_near += 1
            elif v >= 50:
                high_near += 1
    return {
        "size": (w, h),
        "res": res,
        "lethal_all": lethal_all,
        "high_all": high_all,
        "lethal_near": lethal_near,
        "high_near": high_near,
        "unknown_near": unknown_near,
    }


@dataclass
class Sample:
    t: float
    cmd_nav: Tuple[float, float, float]
    cmd_out: Tuple[float, float, float]
    travel: float = 0.0


class DiagNode(Node):
    def __init__(self) -> None:
        super().__init__("diag_nav_obstacle")
        self.odom: Optional[Odometry] = None
        self.cloud: Optional[PointCloud2] = None
        self.local_cm: Optional[OccupancyGrid] = None
        self.global_cm: Optional[OccupancyGrid] = None
        self.plan: Optional[Path] = None
        self.cmd_nav = Twist()
        self.cmd_out = Twist()
        self.create_subscription(Odometry, "/lio/robo/odom", lambda m: setattr(self, "odom", m), ODOM_QOS)
        self.create_subscription(PointCloud2, "/lio/cloud_world", lambda m: setattr(self, "cloud", m), 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: setattr(self, "local_cm", m), 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_global, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_global, MAP_QOS)
        self.create_subscription(Path, "/plan", lambda m: setattr(self, "plan", m), 10)
        self.create_subscription(Twist, "/cmd_vel_nav", lambda m: setattr(self, "cmd_nav", m), 10)
        self.create_subscription(Twist, "/cmd_vel", lambda m: setattr(self, "cmd_out", m), 10)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.samples: List[Sample] = []

    def _on_global(self, m: OccupancyGrid) -> None:
        self.global_cm = m

    def spin_wait(self, sec: float) -> None:
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def record_cmd_sample(self, sx: float, sy: float) -> None:
        if self.odom is None:
            return
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        self.samples.append(
            Sample(
                time.time(),
                (self.cmd_nav.linear.x, self.cmd_nav.linear.y, self.cmd_nav.angular.z),
                (self.cmd_out.linear.x, self.cmd_out.linear.y, self.cmd_out.angular.z),
                math.hypot(px - sx, py - sy),
            )
        )

    def send_goal(self, gx: float, gy: float, gz: float, yaw: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("/navigate_to_pose 不可用")
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation = yaw_to_quat(yaw)
        fut = self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.result() or not fut.result().accepted:
            raise RuntimeError("NavigateToPose 被拒绝")


def tail_controller_log(log_glob: str, lines: int = 200) -> List[str]:
    try:
        import glob

        files = sorted(glob.glob(log_glob), key=os.path.getmtime, reverse=True)
        if not files:
            return []
        with open(files[0], "r", errors="ignore") as f:
            content = f.readlines()
        return content[-lines:]
    except OSError:
        return []


def analyze_log(lines: List[str]) -> dict:
    keys = {
        "progress_fail": len(re.findall(r"Failed to make progress", "\n".join(lines))),
        "rate_miss": len(re.findall(r"Control loop missed its desired rate", "\n".join(lines))),
        "collision": len(re.findall(r"Collision Ahead", "\n".join(lines))),
        "backup_fail": len(re.findall(r"backup failed", "\n".join(lines))),
        "controller_died": len(re.findall(r"controller_server.*process has died", "\n".join(lines))),
    }
    return keys


def summarize_cmd(samples: List[Sample]) -> dict:
    if not samples:
        return {"n": 0}
    nav_nonzero = sum(
        1 for s in samples if abs(s.cmd_nav[0]) + abs(s.cmd_nav[1]) + abs(s.cmd_nav[2]) > 0.02
    )
    out_nonzero = sum(
        1 for s in samples if abs(s.cmd_out[0]) + abs(s.cmd_out[1]) + abs(s.cmd_out[2]) > 0.02
    )
    max_nav = max((abs(s.cmd_nav[0]) for s in samples), default=0.0)
    max_out = max((abs(s.cmd_out[0]) for s in samples), default=0.0)
    return {
        "n": len(samples),
        "nav_nonzero": nav_nonzero,
        "out_nonzero": out_nonzero,
        "max_nav_vx": max_nav,
        "max_out_vx": max_out,
        "final_travel": samples[-1].travel,
    }


def print_verdict(cloud_obs: int, local: dict, global_: dict, plan_lat: float, plan_n: int,
                  cmd: dict, log_stats: dict, had_goal: bool) -> int:
    print("\n======== 诊断结论 ========")
    issues = []
    if cloud_obs == 0:
        issues.append("点云 3m 内无有效障碍高度点 → 纸箱可能未被 lidar 看到或高度过滤不对")
    if local.get("lethal_near", 0) == 0 and local.get("high_near", 0) > 200 and cloud_obs > 20:
        issues.append(
            "局部仅有膨胀高代价、几乎无 lethal → MPPI 可能全程降速/零速；"
            "且动态障未进 global，/plan 无法绕开"
        )
    elif local.get("lethal_near", 0) == 0 and cloud_obs > 20:
        issues.append("有点云但 local 附近 lethal 很少 → 检查 ObstacleLayer 高度/范围")
    if global_.get("lethal_all", 0) > 0 and plan_n > 0 and plan_lat < 0.15 and had_goal:
        issues.append("全局图有障碍但 /plan 几乎直线 → 全局仅静态层、动态障未参与 replan (方案2)")
    if had_goal and cmd.get("nav_nonzero", 0) > 50 and cmd.get("final_travel", 0) < 0.15:
        issues.append(
            "Nav2 持续发 cmd_vel_nav 但 odom 几乎不动 → 优先查机身 obstacles_avoid 是否钳制前进"
            "或原地打滑/被挡"
        )
    if log_stats.get("progress_fail", 0) > 0:
        issues.append(f"controller Failed to make progress ×{log_stats['progress_fail']}")
    if log_stats.get("rate_miss", 0) > 5:
        issues.append(f"控制循环掉帧 ×{log_stats['rate_miss']} → 勿增大 MPPI batch/iter")
    if log_stats.get("collision", 0) > 0:
        issues.append("behavior Collision Ahead → 局部无可行绕行动作")

    if not issues:
        print("未发现单一硬故障；若仍试探，优先实施方案2 (动态障进 global + replan)")
        return 0
    for i, t in enumerate(issues, 1):
        print(f"  {i}. {t}")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--warmup-sec", type=float, default=5.0)
    parser.add_argument("--monitor-sec", type=float, default=15.0)
    parser.add_argument("--navigate-distance", type=float, default=0.0, help=">0 则发送正前方 goal")
    parser.add_argument("--cloud-radius", type=float, default=3.0)
    parser.add_argument("--log-glob", default="/ws/logs/nav_*.log")
    args = parser.parse_args()

    rclpy.init()
    node = DiagNode()
    exit_code = 0
    try:
        print("=== Nav2 障碍/规划/控制诊断 ===")
        node.spin_wait(args.warmup_sec)
        if node.odom is None:
            print("ERROR: 无 /lio/robo/odom", file=sys.stderr)
            return 1

        rx = node.odom.pose.pose.position.x
        ry = node.odom.pose.pose.position.y
        rz = node.odom.pose.pose.position.z
        q = node.odom.pose.pose.orientation
        yaw = quat_yaw(q.x, q.y, q.z, q.w)
        print(f"机器人 world: ({rx:.2f}, {ry:.2f}, {rz:.2f}) yaw={math.degrees(yaw):.1f}°")

        cloud_total = cloud_obs = 0
        if node.cloud is None:
            print("WARN: 未收到 /lio/cloud_world")
        else:
            cloud_total, cloud_obs = count_cloud_near_robot(
                node.cloud, rx, ry, rz, args.cloud_radius, 0.10, 1.50
            )
            print(
                f"点云 {args.cloud_radius}m: 总点 {cloud_total}, "
                f"高度过滤障碍 [0.10,1.50]m 相对地面: {cloud_obs}"
            )

        local = costmap_stats(node.local_cm, rx, ry, 3.0) if node.local_cm else {}
        global_ = costmap_stats(node.global_cm, rx, ry, 3.0) if node.global_cm else {}
        if node.local_cm:
            print(
                f"local costmap {local['size']} lethal_near={local['lethal_near']} "
                f"high(≥100)_near={local['high_near']}"
            )
        else:
            print("WARN: 无 /local_costmap/costmap")
        if node.global_cm:
            print(
                f"global costmap {global_['size']} lethal_all={global_['lethal_all']} "
                f"lethal_near={global_['lethal_near']}"
            )
        else:
            print("WARN: 无 /global_costmap/costmap")

        plan_n = len(node.plan.poses) if node.plan else 0
        plan_lat = 0.0
        gx = gy = 0.0
        had_goal = False
        if args.navigate_distance > 0:
            had_goal = True
            gx = rx + args.navigate_distance * math.cos(yaw)
            gy = ry + args.navigate_distance * math.sin(yaw)
            gz = max(rz, 0.0)
            print(f"发送 goal ({gx:.2f}, {gy:.2f}) 距离 {args.navigate_distance} m, 监测 {args.monitor_sec}s")
            node.send_goal(gx, gy, gz, yaw)
            t0 = time.time()
            while time.time() - t0 < args.monitor_sec:
                rclpy.spin_once(node, timeout_sec=0.05)
                node.record_cmd_sample(rx, ry)
                time.sleep(0.1)
            if node.plan:
                plan_n = len(node.plan.poses)
                plan_lat = lateral_max(rx, ry, gx, gy, node.plan)
        elif node.plan:
            if plan_n >= 2:
                p0 = node.plan.poses[0].pose.position
                p1 = node.plan.poses[-1].pose.position
                plan_lat = lateral_max(p0.x, p0.y, p1.x, p1.y, node.plan)

        print(f"/plan: poses={plan_n}, max_lateral={plan_lat:.2f} m " +
              ("(>0.3 视为明显绕行)" if plan_lat > 0.3 else "(近直线)"))
        if node.plan and len(node.plan.poses) >= 2:
            plen = 0.0
            ps = node.plan.poses
            for i in range(1, len(ps)):
                a, b = ps[i - 1].pose.position, ps[i].pose.position
                plen += math.hypot(b.x - a.x, b.y - a.y)
            straight = args.navigate_distance if args.navigate_distance > 0 else math.hypot(
                ps[-1].pose.position.x - ps[0].pose.position.x,
                ps[-1].pose.position.y - ps[0].pose.position.y,
            )
            ratio = plen / straight if straight > 1e-3 else 0.0
            print(f"/plan 路径长 {plen:.2f} m, 直线 {straight:.2f} m, 比值 {ratio:.2f}" +
                  (" (比值>2 通常不可执行)" if ratio > 2.0 else ""))

        cmd = summarize_cmd(node.samples)
        if had_goal:
            print(
                f"cmd 采样 n={cmd['n']} nav非零={cmd.get('nav_nonzero',0)} "
                f"out非零={cmd.get('out_nonzero',0)} max_vx_nav={cmd.get('max_nav_vx',0):.3f} "
                f"travel={cmd.get('final_travel',0):.2f}m"
            )
        else:
            print("未发 goal；加 --navigate-distance 3 可监测导航中 cmd")

        log_lines = tail_controller_log(args.log_glob)
        log_stats = analyze_log(log_lines)
        print(
            "近期日志统计: "
            + ", ".join(f"{k}={v}" for k, v in log_stats.items())
        )

        exit_code = print_verdict(cloud_obs, local, global_, plan_lat, plan_n, cmd, log_stats, had_goal)
        return exit_code
    finally:
        if args.navigate_distance > 0:
            try:
                subprocess.run(
                    ["ros2", "run", "motionslam_pipeline", "estop_go2"],
                    timeout=8,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
