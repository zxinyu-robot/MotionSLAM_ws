#!/usr/bin/env python3
"""NAV-4: 导航中断网 — 标记断网时刻,观测本地是否继续/停车.

用法 (nav_stack 已启动, with_forwarder:=true):
  python3 scripts/verify_nav4_wifi_disconnect.py --distance 3 --disconnect-at 8

断网方式 (二选一):
  A) 脚本自动: --dev-ip <开发机IP> 用 iptables 阻断到该 IP (需 sudo)
  B) 手动: 到 --disconnect-at 秒时在开发机断 WiFi / 拔网线
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from verify_h5_arrival import H5Node, dist2, quat_rpy, run_estop_cpp, yaw_to_quat  # noqa: E402


def apply_iptables_drop(dev_ip: str) -> bool:
    cmd = ["sudo", "iptables", "-A", "OUTPUT", "-d", dev_ip, "-j", "DROP"]
    try:
        subprocess.run(cmd, check=True, timeout=5)
        return True
    except Exception as e:
        print(f"WARN: iptables 失败 ({e}), 请手动断网", file=sys.stderr)
        return False


def remove_iptables_drop(dev_ip: str) -> None:
    subprocess.run(
        ["sudo", "iptables", "-D", "OUTPUT", "-d", dev_ip, "-j", "DROP"],
        check=False,
        timeout=5,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="NAV-4 WiFi/远程链路中断")
    parser.add_argument("--distance", type=float, default=3.0)
    parser.add_argument("--arrive-dist", type=float, default=0.3)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--disconnect-at", type=float, default=8.0, help="发 goal 后断网时刻 (s)")
    parser.add_argument("--dev-ip", default="", help="开发机 IP, 自动 iptables 阻断")
    parser.add_argument("--marker-file", default="", help="断网时刻写入此文件")
    parser.add_argument("--odom-topic", default="/lio/robo/odom")
    args = parser.parse_args()

    marker_path = args.marker_file or f"/tmp/nav4_disconnect_{int(time.time())}.txt"

    rclpy.init()
    node = H5Node(args.odom_topic)
    iptables_applied = False

    try:
        print("NAV-4: 等待 odom ...")
        t0 = time.time()
        while node.latest is None and time.time() - t0 < 30:
            node.spin_once()
        if node.latest is None:
            print("ERROR: 无 odom", file=sys.stderr)
            return 1

        ox = node.latest.pose.pose.position.x
        oy = node.latest.pose.pose.position.y
        _, _, yaw = quat_rpy(
            node.latest.pose.pose.orientation.x,
            node.latest.pose.pose.orientation.y,
            node.latest.pose.pose.orientation.z,
            node.latest.pose.pose.orientation.w,
        )
        gx = ox + args.distance * math.cos(yaw)
        gy = oy + args.distance * math.sin(yaw)
        gz = max(node.latest.pose.pose.position.z, 0.0)

        print(f"  起点 ({ox:.2f}, {oy:.2f}) → goal ({gx:.2f}, {gy:.2f})")
        print(f"  t={args.disconnect_at}s 时断网 (dev-ip={args.dev_ip or '手动'})")

        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation = yaw_to_quat(yaw)

        handle = node.send_nav2_goal(goal)
        result_future = handle.get_result_async()
        start = time.time()
        disconnected = False
        conclusion = "unknown"

        while time.time() - start < args.timeout:
            node.spin_once()
            elapsed = time.time() - start

            if not disconnected and elapsed >= args.disconnect_at:
                disconnected = True
                line = f"DISCONNECT_MARK t={elapsed:.2f}s wall={time.strftime('%Y-%m-%dT%H:%M:%S')}\n"
                print(f"  >>> {line.strip()}")
                with open(marker_path, "w", encoding="utf-8") as f:
                    f.write(line)
                    if args.dev_ip:
                        f.write(f"dev_ip={args.dev_ip}\n")
                if args.dev_ip:
                    iptables_applied = apply_iptables_drop(args.dev_ip)
                    with open(marker_path, "a", encoding="utf-8") as f:
                        f.write(f"iptables={'ok' if iptables_applied else 'fail'}\n")
                else:
                    print("  >>> 请现在在开发机/办公网断 WiFi 或拔网线 <<<")

            if node.latest is None:
                time.sleep(0.05)
                continue

            px = node.latest.pose.pose.position.x
            py = node.latest.pose.pose.position.y
            d_goal = dist2(px, py, gx, gy)
            travel = dist2(px, py, ox, oy)

            if d_goal <= args.arrive_dist:
                conclusion = "本地完成"
                print(f"PASS [{conclusion}]: 误差 {d_goal:.3f} m, 行走 {travel:.2f} m, {elapsed:.1f}s")
                with open(marker_path, "a", encoding="utf-8") as f:
                    f.write(f"conclusion={conclusion}\n")
                return 0

            if result_future.done():
                break
            time.sleep(0.1)

        px = node.latest.pose.pose.position.x if node.latest else ox
        py = node.latest.pose.pose.position.y if node.latest else oy
        d_goal = dist2(px, py, gx, gy)
        travel = dist2(px, py, ox, oy) if node.latest else 0.0

        # 断网后仍移动 >0.5m 但未到达 → 部分本地；几乎不动 → 安全停车
        if travel >= 1.0 and d_goal > args.arrive_dist:
            conclusion = "本地部分完成/未到达"
        elif travel < 0.3:
            conclusion = "安全停车"
        else:
            conclusion = "中止/待人工判定"

        print(f"PARTIAL [{conclusion}]: 距 goal {d_goal:.2f} m, 行走 {travel:.2f} m")
        with open(marker_path, "a", encoding="utf-8") as f:
            f.write(f"conclusion={conclusion}\n")
        return 2 if conclusion == "本地部分完成/未到达" else 1

    finally:
        if iptables_applied and args.dev_ip:
            remove_iptables_drop(args.dev_ip)
        print("  断网后 estop 探针 ...")
        run_estop_cpp()
        time.sleep(1.0)
        print("  estop_go2 已调用 (obstacles_avoid 仍由本地 Sport 栈处理)")
        try:
            node.safe_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
