#!/usr/bin/env python3
"""RGB → BEV 语义栅格（Step 1c）。默认不做 stub 推理；真推理走 TensorRT C++ `bev_glass_layer`."""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Header

try:
    from unitree_go.msg import Go2FrontVideoData
except ImportError:
    Go2FrontVideoData = None  # type: ignore[misc, assignment]


def _yaw_from_quat(qz: float, qw: float) -> float:
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def _stub_bev_from_payload(payload: bytes, width: int, height: int) -> list[int]:
    """按 payload 字节分布生成伪语义占据（0=free, 100=occupied）."""
    if not payload:
        return [0] * (width * height)
    cells: list[int] = []
    chunk = max(1, len(payload) // (width * height))
    for row in range(height):
        for col in range(width):
            idx = (row * width + col) * chunk
            window = payload[idx : idx + chunk]
            if not window:
                cells.append(0)
                continue
            mean = sum(window) / len(window)
            cells.append(100 if mean > 140.0 else 0)
    return cells


class RgbBevNode(Node):
    def __init__(self) -> None:
        super().__init__("rgb_bev_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("allow_stub", False)
        self.declare_parameter("video_topic", "/frontvideostream")
        self.declare_parameter("stream_field", "video360p")
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("output_topic", "/semantic/bev_grid_world")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("grid_width", 32)
        self.declare_parameter("grid_height", 32)
        self.declare_parameter("resolution", 0.15)
        self.declare_parameter("grid_depth_m", 4.8)
        self.declare_parameter("grid_width_m", 4.8)
        self.declare_parameter("process_every_n", 3)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._allow_stub = bool(self.get_parameter("allow_stub").value)
        self._stream_field = str(self.get_parameter("stream_field").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._grid_w = int(self.get_parameter("grid_width").value)
        self._grid_h = int(self.get_parameter("grid_height").value)
        self._res = float(self.get_parameter("resolution").value)
        self._depth_m = float(self.get_parameter("grid_depth_m").value)
        self._width_m = float(self.get_parameter("grid_width_m").value)
        self._every_n = max(1, int(self.get_parameter("process_every_n").value))

        self._odom: Optional[Odometry] = None
        self._frame_count = 0
        self._pub_seq = 0

        odom_topic = str(self.get_parameter("odom_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._grid_pub = self.create_publisher(OccupancyGrid, out_topic, 10)

        if not self._enabled:
            self.get_logger().warn("rgb_bev_node disabled")
            return
        if Go2FrontVideoData is None:
            self.get_logger().error("unitree_go 未安装，无法订阅前向相机")
            return

        if not self._allow_stub:
            self.get_logger().warn(
                "rgb_bev_node: stub 推理已关闭。真 BEV 使用 TensorRT C++ bev_glass_layer"
            )
            return
        video_topic = str(self.get_parameter("video_topic").value)
        self.create_subscription(Go2FrontVideoData, video_topic, self._on_video, 10)
        self.get_logger().info(
            f"RGB→BEV stub (dev only): {video_topic} → {out_topic} "
            f"({self._grid_w}x{self._grid_h} @ {self._res}m, frame={self._frame_id})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_video(self, msg: Go2FrontVideoData) -> None:
        if not self._enabled or self._odom is None:
            return
        self._frame_count += 1
        if self._frame_count % self._every_n != 0:
            return

        payload = bytes(getattr(msg, self._stream_field, b"") or b"")
        if len(payload) < 32:
            return

        odom = self._odom.pose.pose
        yaw = _yaw_from_quat(odom.orientation.z, odom.orientation.w)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # 栅格锚点在机身前方 grid_depth_m/2，ENU 对齐
        cx = odom.position.x + (self._depth_m * 0.5) * cos_y
        cy = odom.position.y + (self._depth_m * 0.5) * sin_y
        origin_x = cx - 0.5 * self._width_m * cos_y + 0.5 * self._width_m * sin_y
        origin_y = cy - 0.5 * self._width_m * sin_y - 0.5 * self._width_m * cos_y

        data = _stub_bev_from_payload(payload, self._grid_w, self._grid_h)
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._frame_id
        grid.info.resolution = self._res
        grid.info.width = self._grid_w
        grid.info.height = self._grid_h
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = max(odom.position.z, 0.0)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        grid.info.origin.orientation.z = qz
        grid.info.origin.orientation.w = qw
        grid.data = data

        self._grid_pub.publish(grid)
        self._pub_seq += 1
        if self._pub_seq == 1 or self._pub_seq % 30 == 0:
            self.get_logger().info(
                f"BEV grid #{self._pub_seq} origin=({origin_x:.2f},{origin_y:.2f}) "
                f"occupied={sum(1 for v in data if v > 50)}"
            )


def main() -> None:
    rclpy.init()
    node = RgbBevNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
