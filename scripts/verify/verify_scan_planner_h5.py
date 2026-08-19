#!/usr/bin/env python3
"""H5 端到端到达 (SCAN-Planner): 发 /move_base_simple/goal, 监测 odom 是否进到达半径.

用法 (demo_scan_stack + forwarder, 空场):
  python3 scripts/verify/verify_scan_planner_h5.py --distance 3
  python3 scripts/verify/verify_scan_planner_h5.py --distance 3 --dry-run
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from motionslam_msgs.msg import MotionCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from h5_arrival_utils import (  # noqa: E402
    ODOM_QOS,
    dist2,
    quat_rpy,
    run_estop_cpp,
    yaw_to_quat,
    zero_motion_command,
)


def normalize_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def yaw_delta(from_yaw: float, to_yaw: float) -> float:
    return normalize_yaw(to_yaw - from_yaw)


class ScanPlannerH5Node(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("verify_scan_planner_h5")
        self._latest: Odometry | None = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, ODOM_QOS)
        self._goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self._motion_pub = self.create_publisher(MotionCommand, "/motion/command", 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest = msg

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.05)

    @property
    def latest(self) -> Odometry | None:
        return self._latest

    def send_goal(self, goal: PoseStamped) -> None:
        for _ in range(3):
            self._goal_pub.publish(goal)
            self.spin_once()
            time.sleep(0.05)

    def safe_stop(self) -> None:
        print("  安全停车: 零速 + estop...")
        zero = zero_motion_command()
        for _ in range(5):
            zero.header.stamp = self.get_clock().now().to_msg()
            self._motion_pub.publish(zero)
            self.spin_once()
            time.sleep(0.05)
        run_estop_cpp()

    def turn_in_place(
        self,
        turn_deg: float,
        *,
        timeout: float,
        tolerance_deg: float,
        angular_z: float,
    ) -> bool:
        if abs(turn_deg) < 1.0:
            return True
        if self._latest is None:
            return False
        ox = self._latest.pose.pose.position.x
        oy = self._latest.pose.pose.position.y
        q0 = self._latest.pose.pose.orientation
        start_yaw = quat_rpy(q0.x, q0.y, q0.z, q0.w)[2]
        target_yaw = normalize_yaw(start_yaw + math.radians(turn_deg))
        tol = math.radians(tolerance_deg)
        gz = max(self._latest.pose.pose.position.z, 0.3)

        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = self.get_clock().now().to_msg()
        # 略向后偏移，避免 planner 判定已在 goal 而忽略朝向
        back_yaw = normalize_yaw(start_yaw + math.radians(turn_deg))
        offset = 0.5
        goal.pose.position.x = ox + offset * math.cos(back_yaw)
        goal.pose.position.y = oy + offset * math.sin(back_yaw)
        goal.pose.position.z = gz
        goal.pose.orientation = yaw_to_quat(back_yaw)

        print(
            f"  原地调头 {turn_deg:+.0f}° via scan_planner "
            f"(tol={tolerance_deg}°)..."
        )
        self.send_goal(goal)

        t0 = time.time()
        max_drift = 0.0
        while time.time() - t0 < timeout:
            self.spin_once()
            if self._latest is None:
                time.sleep(0.05)
                continue
            px = self._latest.pose.pose.position.x
            py = self._latest.pose.pose.position.y
            q = self._latest.pose.pose.orientation
            yaw = quat_rpy(q.x, q.y, q.z, q.w)[2]
            delta = yaw_delta(start_yaw, yaw)
            max_drift = max(max_drift, dist2(px, py, ox, oy))
            target_delta = math.radians(turn_deg)
            turn_err = min(
                abs(normalize_yaw(delta - target_delta)),
                abs(normalize_yaw(delta + target_delta)),
            )
            if turn_err <= tol and max_drift <= 1.5:
                print(
                    f"  调头完成: Δyaw={math.degrees(delta):+.1f}° "
                    f"漂移={max_drift:.2f}m ({time.time() - t0:.1f}s)"
                )
                time.sleep(0.5)
                return True
            time.sleep(0.1)

        if self._latest is not None:
            q = self._latest.pose.pose.orientation
            yaw = quat_rpy(q.x, q.y, q.z, q.w)[2]
            delta = yaw_delta(start_yaw, yaw)
            print(
                f"ERROR: 原地调头超时 Δyaw={math.degrees(delta):+.1f}° "
                f"漂移={max_drift:.2f}m",
                file=sys.stderr,
            )
        else:
            print("ERROR: 原地调头超时", file=sys.stderr)
        return False


def walk_to_goal(
    node: ScanPlannerH5Node,
    *,
    ox: float,
    oy: float,
    gx: float,
    gy: float,
    gz: float,
    yaw: float,
    arrive_dist: float,
    timeout: float,
    label: str,
) -> bool:
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.position.z = gz
    goal.pose.orientation = yaw_to_quat(yaw)
    print(f"  [{label}] goal ({gx:.2f}, {gy:.2f}) 距离 {dist2(ox, oy, gx, gy):.2f} m")
    node.send_goal(goal)

    start = time.time()
    best = float("inf")
    while time.time() - start < timeout:
        node.spin_once()
        if node.latest is None:
            time.sleep(0.05)
            continue
        px = node.latest.pose.pose.position.x
        py = node.latest.pose.pose.position.y
        d_goal = dist2(px, py, gx, gy)
        best = min(best, d_goal)
        if d_goal <= arrive_dist:
            walked = dist2(px, py, ox, oy)
            print(
                f"  [{label}] OK 误差 {d_goal:.3f} m, 行走 {walked:.2f} m, "
                f"{time.time() - start:.1f}s"
            )
            return True
        time.sleep(0.1)

    px = node.latest.pose.pose.position.x if node.latest else ox
    py = node.latest.pose.pose.position.y if node.latest else oy
    d_goal = dist2(px, py, gx, gy)
    walked = dist2(px, py, ox, oy)
    print(
        f"  [{label}] FAIL 距 goal {d_goal:.3f} m (最近 {best:.3f} m), "
        f"行走 {walked:.2f} m",
        file=sys.stderr,
    )
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="SCAN-Planner H5 到达验收")
    parser.add_argument("--distance", type=float, default=3.0)
    parser.add_argument("--arrive-dist", type=float, default=0.35)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--odom-topic", default="/lio/robo/odom")
    parser.add_argument("--wait-odom", type=float, default=45.0)
    parser.add_argument("--yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--turn-deg", type=float, default=0.0, help="先原地旋转再直走")
    parser.add_argument("--turn-timeout", type=float, default=35.0)
    parser.add_argument("--turn-tolerance-deg", type=float, default=10.0)
    parser.add_argument("--turn-angular", type=float, default=0.55)
    parser.add_argument("--settle-after-turn-s", type=float, default=3.0)
    parser.add_argument("--segment-m", type=float, default=0.0, help=">0 时分段直走")
    parser.add_argument("--approach-m", type=float, default=0.0, help="先直走到开阔区 (m)")
    parser.add_argument(
        "--goal-x",
        type=float,
        default=None,
        help="绝对 goal X (world)；与 --goal-y 合用，用于穿门/进房间",
    )
    parser.add_argument("--goal-y", type=float, default=None, help="绝对 goal Y (world)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = ScanPlannerH5Node(args.odom_topic)
    exit_code = 1

    try:
        print(f"SCAN-Planner H5: 等待 {args.odom_topic} (最多 {args.wait_odom:.0f}s)...")
        t0 = time.time()
        while node.latest is None and time.time() - t0 < args.wait_odom:
            node.spin_once()
        if node.latest is None:
            print("ERROR: 无 odom", file=sys.stderr)
            return 1

        ox = node.latest.pose.pose.position.x
        oy = node.latest.pose.pose.position.y
        q = node.latest.pose.pose.orientation
        roll, pitch, yaw_odom = quat_rpy(q.x, q.y, q.z, q.w)
        yaw = yaw_odom + math.radians(args.yaw_offset_deg)
        gz = max(node.latest.pose.pose.position.z, 0.3)

        use_absolute_goal = args.goal_x is not None and args.goal_y is not None
        if use_absolute_goal:
            final_gx = float(args.goal_x)
            final_gy = float(args.goal_y)
            total_dist = dist2(ox, oy, final_gx, final_gy)
            gx, gy = final_gx, final_gy
        else:
            final_gx = ox + args.distance * math.cos(yaw)
            final_gy = oy + args.distance * math.sin(yaw)
            total_dist = args.distance
            gx, gy = final_gx, final_gy

        print(
            f"  当前 ({ox:.2f}, {oy:.2f}) yaw={math.degrees(yaw_odom):.1f}° "
            f"[{args.odom_topic}]"
        )
        if use_absolute_goal:
            print(
                f"  goal ({gx:.2f}, {gy:.2f}, {gz:.2f}) "
                f"绝对目标 距离 {total_dist:.2f} m"
            )
        else:
            print(f"  goal ({gx:.2f}, {gy:.2f}, {gz:.2f}) 距离 {args.distance} m")
        print(f"  到达半径 {args.arrive_dist} m, 超时 {args.timeout}s")

        if args.dry_run:
            if args.approach_m > 0:
                print(f"  [dry-run] 将先前进 {args.approach_m} m")
            if abs(args.turn_deg) >= 1.0:
                print(f"  [dry-run] 将先原地调头 {args.turn_deg:+.0f}°")
            return 0

        if args.approach_m > 0:
            print(f"  先前进 {args.approach_m} m 到开阔区...")
            agx = ox + args.approach_m * math.cos(yaw)
            agy = oy + args.approach_m * math.sin(yaw)
            if not walk_to_goal(
                node,
                ox=ox,
                oy=oy,
                gx=agx,
                gy=agy,
                gz=gz,
                yaw=yaw,
                arrive_dist=args.arrive_dist,
                timeout=max(60.0, args.timeout * 0.4),
                label="approach",
            ):
                return 1
            time.sleep(2.0)
            node.spin_once()
            if node.latest is None:
                return 1
            ox = node.latest.pose.pose.position.x
            oy = node.latest.pose.pose.position.y
            q = node.latest.pose.pose.orientation
            roll, pitch, yaw_odom = quat_rpy(q.x, q.y, q.z, q.w)
            yaw = yaw_odom + math.radians(args.yaw_offset_deg)
            gz = max(node.latest.pose.pose.position.z, 0.3)
            print(
                f"  开阔区 ({ox:.2f}, {oy:.2f}) yaw={math.degrees(yaw_odom):.1f}°"
            )

        if abs(args.turn_deg) >= 1.0:
            if not node.turn_in_place(
                args.turn_deg,
                timeout=args.turn_timeout,
                tolerance_deg=args.turn_tolerance_deg,
                angular_z=args.turn_angular,
            ):
                return 1
            time.sleep(0.5)
            node.spin_once()
            if node.latest is None:
                print("ERROR: 调头后无 odom", file=sys.stderr)
                return 1
            ox = node.latest.pose.pose.position.x
            oy = node.latest.pose.pose.position.y
            q = node.latest.pose.pose.orientation
            roll, pitch, yaw_odom = quat_rpy(q.x, q.y, q.z, q.w)
            yaw = yaw_odom + math.radians(args.yaw_offset_deg)
            gz = max(node.latest.pose.pose.position.z, 0.3)
            gx = ox + args.distance * math.cos(yaw)
            gy = oy + args.distance * math.sin(yaw)
            if use_absolute_goal:
                final_gx, final_gy = float(args.goal_x), float(args.goal_y)
                gx, gy = final_gx, final_gy
            print(
                f"  调头后 ({ox:.2f}, {oy:.2f}) yaw={math.degrees(yaw_odom):.1f}°"
            )
            print(f"  goal ({gx:.2f}, {gy:.2f}, {gz:.2f}) 直走 {args.distance} m")
            if args.settle_after_turn_s > 0:
                print(f"  调头后 settle {args.settle_after_turn_s:.1f}s...")
                time.sleep(args.settle_after_turn_s)
                node.spin_once()

        print("  发送 /move_base_simple/goal — 狗将移动, 请确认 with_forwarder:=true")
        seg = args.segment_m if args.segment_m > 0 else total_dist
        n_seg = max(1, int(math.ceil(total_dist / seg)))
        per_timeout = max(30.0, args.timeout / n_seg)

        for i in range(n_seg):
            node.spin_once()
            if node.latest is None:
                return 1
            leg_ox = node.latest.pose.pose.position.x
            leg_oy = node.latest.pose.pose.position.y
            remain = dist2(leg_ox, leg_oy, final_gx, final_gy)
            if remain <= args.arrive_dist:
                print(f"  已到达最终 goal, 剩余 {remain:.3f} m")
                break
            step = min(seg, remain)
            if use_absolute_goal:
                ratio = step / remain if remain > 1e-6 else 1.0
                gx = leg_ox + ratio * (final_gx - leg_ox)
                gy = leg_oy + ratio * (final_gy - leg_oy)
                leg_yaw = math.atan2(final_gy - leg_oy, final_gx - leg_ox)
            else:
                gx = leg_ox + step * math.cos(yaw)
                gy = leg_oy + step * math.sin(yaw)
                leg_yaw = yaw
            if not walk_to_goal(
                node,
                ox=leg_ox,
                oy=leg_oy,
                gx=gx,
                gy=gy,
                gz=gz,
                yaw=leg_yaw,
                arrive_dist=args.arrive_dist,
                timeout=per_timeout,
                label=f"leg {i + 1}/{n_seg}",
            ):
                return 1
            if i + 1 < n_seg:
                time.sleep(1.5)
                node.spin_once()

        print(f"OK: 到达目标 ({final_gx:.2f}, {final_gy:.2f}) ({n_seg} 段)")
        return 0
    finally:
        try:
            node.safe_stop()
        except Exception as e:
            print(f"WARN: safe_stop 异常: {e}", file=sys.stderr)
            run_estop_cpp()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
