#!/usr/bin/env python3
"""真机联调：实时打印 glass_suspect summary + 虚拟障碍 + BT event."""
from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class GlassWatch(Node):
    def __init__(self) -> None:
        super().__init__("glass_watch")
        self._last_summary: dict = {}
        self._virtual_count = 0
        self.create_subscription(String, "/perception/glass_suspect/summary", self._on_summary, 10)
        self.create_subscription(PointCloud2, "/perception/virtual_obstacles", self._on_virtual, 10)
        self.create_subscription(String, "/demo/mission/event", self._on_event, 10)
        self.create_timer(2.0, self._print_status)
        self.get_logger().info("watching glass topics (Ctrl+C 退出)")

    def _on_summary(self, msg: String) -> None:
        try:
            self._last_summary = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_virtual(self, msg: PointCloud2) -> None:
        self._virtual_count = int(msg.width) * int(msg.height)

    def _on_event(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "glass" in ev.get("event", "").lower():
            print(f"[BT] {ev}", flush=True)

    def _print_status(self) -> None:
        s = self._last_summary
        if not s:
            print("[glass] 等待 /perception/glass_suspect/summary ...", flush=True)
            return
        print(
            "[glass] "
            f"state={s.get('state')} suspect={s.get('glass_suspect')} "
            f"confirmed={s.get('glass_confirmed')} "
            f"occl={s.get('occlusion_violations')} spec={s.get('specular_score')} "
            f"tau_front={s.get('front_max_tau_nm')} patches={s.get('solid_patch_count', 0)} "
            f"reasons={s.get('reasons')} virtual_pts={self._virtual_count}",
            flush=True,
        )


def main() -> None:
    rclpy.init()
    node = GlassWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
