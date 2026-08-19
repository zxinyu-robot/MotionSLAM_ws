#!/usr/bin/env python3
"""GO2 Nav2 栈 L1–L4 开机自检。

用法 (容器内, 导航已 launch):
  python3 /ws/scripts/check_nav_stack.py
  python3 /ws/scripts/check_nav_stack.py --no-plan
  python3 /ws/scripts/check_nav_stack.py --plan-distance 1.5

退出码: 0=全部关键项通过, 1=有 FAIL
详见 docs/运维/Demo阶段3D_runbook.md
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

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


@dataclass
class CheckResult:
    layer: str
    name: str
    ok: bool
    detail: str


@dataclass
class Report:
    results: List[CheckResult] = field(default_factory=list)

    def add(self, layer: str, name: str, ok: bool, detail: str) -> None:
        self.results.append(CheckResult(layer, name, ok, detail))
        mark = "PASS" if ok else "FAIL"
        print(f"[{mark}] {layer} {name}: {detail}")

    @property
    def all_ok(self) -> bool:
        return all(r.ok for r in self.results)


class NavStackChecker(Node):
    def __init__(self) -> None:
        super().__init__("check_nav_stack")
        self._odom: Optional[Odometry] = None
        self._odom_times: List[float] = []
        self._costmap: Optional[OccupancyGrid] = None
        # 与 TF base_link / H5 一致 (机身系); /lio/odom 仍带雷达 pitch
        self.create_subscription(Odometry, "/lio/robo/odom", self._on_odom, ODOM_QOS)
        # costmap 多为 VOLATILE；同时兼容 TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap, 10
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap, MAP_QOS
        )
        self._nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_times.append(time.time())
        if len(self._odom_times) > 40:
            self._odom_times = self._odom_times[-40:]

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def spin_for(self, seconds: float) -> None:
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def odom_hz(self) -> float:
        ts = self._odom_times
        if len(ts) < 3:
            return 0.0
        dt = ts[-1] - ts[0]
        return (len(ts) - 1) / dt if dt > 1e-3 else 0.0


def quat_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def check_tf(node: Node, timeout: float = 3.0) -> Tuple[bool, str]:
    try:
        from tf2_ros import Buffer, TransformListener
    except ImportError:
        return False, "tf2_ros 不可用"

    buf = Buffer()
    TransformListener(buf, node)
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = buf.lookup_transform("world", "base_link", rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            return True, f"world→base_link ok, xy=({x:.2f},{y:.2f})"
        except Exception:
            pass
    return False, "timeout: 无 world→base_link（检查 robot_pose_viz）"


def node_exists(node: Node, name: str) -> bool:
    names = node.get_node_names()
    return any(n == name or n.endswith(name) for n in names)


def main() -> int:
    parser = argparse.ArgumentParser(description="GO2 Nav2 L1–L4 开机自检")
    parser.add_argument("--wait-odom", type=float, default=15.0)
    parser.add_argument("--min-odom-hz", type=float, default=5.0)
    parser.add_argument("--min-costmap-dim", type=int, default=200)
    parser.add_argument("--shm-name", default="motionslam_map")
    parser.add_argument(
        "--no-plan",
        action="store_true",
        help="兼容旧调用；NavigateToPose 自检不会发送运动目标",
    )
    parser.add_argument("--plan-distance", type=float, default=1.5)
    parser.add_argument("--plan-timeout", type=float, default=15.0)
    args = parser.parse_args()

    report = Report()
    rclpy.init()
    node = NavStackChecker()

    print("=== L1 定位 ===")
    print(f"等待 /lio/robo/odom ≤{args.wait_odom:.0f}s ...")
    t0 = time.time()
    while node._odom is None and time.time() - t0 < args.wait_odom:
        node.spin_for(0.2)
    if node._odom is None:
        report.add("L1", "odom", False, "无 /lio/robo/odom")
    else:
        node.spin_for(1.5)
        hz = node.odom_hz()
        p = node._odom.pose.pose.position
        yaw = math.degrees(quat_yaw(node._odom.pose.pose.orientation))
        report.add(
            "L1",
            "odom",
            hz >= args.min_odom_hz,
            f"{hz:.1f} Hz, pose=({p.x:.2f},{p.y:.2f}), yaw={yaw:.1f}°",
        )

    ok_tf, tf_detail = check_tf(node)
    report.add("L1", "tf", ok_tf, tf_detail)

    print("=== L2 地图/代价图 ===")
    shm_path = f"/dev/shm/{args.shm_name}"
    if os.path.exists(shm_path):
        sz = os.path.getsize(shm_path)
        report.add("L2", "shm", sz > 1024, f"{shm_path} size={sz}")
    else:
        report.add("L2", "shm", False, f"缺失 {shm_path}")

    node.spin_for(2.0)
    if node._costmap is None:
        report.add("L2", "costmap", False, "无 /global_costmap/costmap")
    else:
        w, h = node._costmap.info.width, node._costmap.info.height
        res = node._costmap.info.resolution
        ok = w >= args.min_costmap_dim and h >= args.min_costmap_dim
        report.add(
            "L2",
            "costmap",
            ok,
            f"{w}x{h} @ {res:.3f}m frame={node._costmap.header.frame_id}",
        )

    print("=== L3 NavigateToPose ===")
    has_bt = node_exists(node, "bt_navigator")
    has_planner = node_exists(node, "planner_server")
    has_ctrl = node_exists(node, "controller_server")
    report.add(
        "L3",
        "nodes",
        has_bt and has_planner and has_ctrl,
        f"bt_navigator={has_bt}, planner_server={has_planner}, "
        f"controller_server={has_ctrl}",
    )
    nav_ready = node._nav_client.wait_for_server(timeout_sec=5.0)
    report.add(
        "L3",
        "navigate_action",
        nav_ready,
        "/navigate_to_pose action 可用（自检不发送运动目标）"
        if nav_ready
        else "/navigate_to_pose action 不可用",
    )

    print("=== L4 控制链路 ===")
    has_fwd = node_exists(node, "cmd_vel_forwarder")
    report.add(
        "L4",
        "controller",
        has_ctrl,
        f"controller_server={has_ctrl}",
    )
    if has_fwd:
        report.add("L4", "forwarder", True, "cmd_vel_forwarder 在线（可真机下发）")
    else:
        report.add(
            "L4",
            "forwarder",
            True,
            "未检测到 forwarder（干跑 OK；真机请 with_forwarder:=true）",
        )

    cmd_publishers = node.get_publishers_info_by_topic("/cmd_vel")
    cmd_nodes = [info.node_name for info in cmd_publishers]
    unique_cmd = len(cmd_publishers) == 1 and cmd_nodes[0] == "controller_server"
    report.add(
        "L4",
        "cmd_vel_owner",
        unique_cmd,
        f"publishers={cmd_nodes}; 要求唯一 controller_server",
    )

    print()
    print("======== 汇总 ========")
    # L4 forwarder 警告不算硬失败；其余 FAIL 计失败
    hard_fails = [
        r
        for r in report.results
        if not r.ok and not (r.layer == "L4" and r.name == "forwarder")
    ]
    if not hard_fails:
        print("OK: L1–L4 关键项通过，可继续 H5 / 真机测试")
        if not has_fwd:
            print("NOTE: 当前无 cmd_vel_forwarder，真机前请 with_forwarder:=true")
        code = 0
    else:
        print(f"FAIL: {len(hard_fails)} 项未通过")
        for r in hard_fails:
            print(f"  - {r.layer} {r.name}: {r.detail}")
        print("提示: 见 docs/运维/Demo阶段3D_runbook.md 分层处理")
        code = 1

    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
