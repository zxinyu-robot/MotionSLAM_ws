#!/usr/bin/env python3
"""一次性补发 Demo lifecycle autostart（延迟启动 LIO 后用）."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DemoScanLifecycleAutostart(Node):
    def __init__(self) -> None:
        super().__init__("demo_scan_lifecycle_autostart_once")
        self._pub = self.create_publisher(String, "/demo/lifecycle/transition", 10)
        self._start = self.create_publisher(String, "/demo/mission/start", 10)
        self._timer = self.create_timer(1.0, self._fire_once)
        self._done = False

    def _fire_once(self) -> None:
        if self._done:
            return
        self._done = True
        for cmd in ("configure", "activate"):
            msg = String()
            msg.data = cmd
            self._pub.publish(msg)
        start = String()
        start.data = "start"
        self._start.publish(start)
        self.get_logger().info("已补发 configure + activate + mission/start")
        self._timer.cancel()


def main() -> None:
    rclpy.init()
    node = DemoScanLifecycleAutostart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
