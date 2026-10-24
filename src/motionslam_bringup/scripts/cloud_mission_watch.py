#!/usr/bin/env python3
"""云端闭环冒烟观察器：订阅 /demo/mission/event，mission_succeeded → exit 0。

用法: ros2 run motionslam_bringup cloud_mission_watch.py --ros-args -p timeout_s:=240
"""
from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CloudMissionWatch(Node):
    def __init__(self) -> None:
        super().__init__("cloud_mission_watch")
        self.declare_parameter("timeout_s", 240.0)
        self.declare_parameter("mission_event_topic", "/demo/mission/event")
        self._timeout = float(self.get_parameter("timeout_s").value)
        self._start = time.monotonic()
        self._result = None
        self.create_subscription(
            String,
            str(self.get_parameter("mission_event_topic").value),
            self._on_event,
            20,
        )
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"watching mission events (timeout={self._timeout}s)")

    def _on_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ev = body.get("event", "")
        detail = body.get("detail", "")
        print(f"[watch] {ev} {detail} idx={body.get('subgoal_index')}", flush=True)
        if ev == "mission_succeeded":
            self._result = 0
        elif ev in ("mission_failed", "mission_rejected"):
            self._result = 1

    def _tick(self) -> None:
        if self._result is not None:
            print(f"[watch] DONE rc={self._result}", flush=True)
            rclpy.shutdown()
            return
        if time.monotonic() - self._start > self._timeout:
            print("[watch] TIMEOUT", flush=True)
            self._result = 2
            rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = CloudMissionWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rc = node._result if node._result is not None else 3
    node.destroy_node()
    return rc


if __name__ == "__main__":
    sys.exit(main())
