#!/usr/bin/env python3
"""语义 BEV 栅格 Token 上行 (:9876)."""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from data_layer_types import DataLayerId  # noqa: E402
from edge_uplink_client import EdgeTcpUplinkClient  # noqa: E402
from semantic_bev_frame import (  # noqa: E402
    SemanticTokenMetaInput,
    build_semantic_token_meta,
    pack_semantic_token_frame,
)
from subgraph_snapshot import quat_yaw  # noqa: E402


class SemanticTokenUplinkNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_token_uplink")
        self.declare_parameter("enabled", True)
        self.declare_parameter("edge_host", "127.0.0.1")
        self.declare_parameter("token_port", 9876)
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("tile_id", "main")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("bev_topic", "/semantic/bev_grid_world")
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("layer_event_topic", "/edge/data_layer/layer_event")
        self.declare_parameter("queue_max", 4)
        self.declare_parameter("connect_timeout_s", 2.0)
        self.declare_parameter("send_timeout_s", 5.0)
        self.declare_parameter("min_grid_cells", 16)
        self.declare_parameter("log_every_n", 10)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._session_id = str(self.get_parameter("session_id").value)
        self._floor_id = str(self.get_parameter("floor_id").value)
        self._tile_id = str(self.get_parameter("tile_id").value)
        self._map_id = str(self.get_parameter("map_id").value)
        self._min_cells = int(self.get_parameter("min_grid_cells").value)
        self._log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self._odom: Optional[Odometry] = None
        self._context_generation = 0
        self._seq = 0
        self._client: Optional[EdgeTcpUplinkClient] = None

        if self._enabled:
            self._client = EdgeTcpUplinkClient(
                host=str(self.get_parameter("edge_host").value),
                port=int(self.get_parameter("token_port").value),
                queue_max=int(self.get_parameter("queue_max").value),
                connect_timeout_s=float(self.get_parameter("connect_timeout_s").value),
                send_timeout_s=float(self.get_parameter("send_timeout_s").value),
                on_error=lambda err: self.get_logger().warn(f"token uplink: {err}"),
                binary_packer=pack_semantic_token_frame,
            )

        odom_topic = str(self.get_parameter("odom_topic").value)
        ctx_topic = str(self.get_parameter("context_generation_topic").value)
        bev_topic = str(self.get_parameter("bev_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(UInt64, ctx_topic, self._on_context_generation, 10)
        self.create_subscription(OccupancyGrid, bev_topic, self._on_bev_grid, 10)
        self._event_pub = self.create_publisher(String, "/edge/uplink/token_event", 10)
        self.get_logger().info(
            f"Token uplink {bev_topic} → :{int(self.get_parameter('token_port').value)} "
            f"schema=motionslam.spatial_semantic_token.v1"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_context_generation(self, msg: UInt64) -> None:
        self._context_generation = int(msg.data)

    def _on_bev_grid(self, msg: OccupancyGrid) -> None:
        if not self._enabled or self._client is None:
            return
        if len(msg.data) < self._min_cells:
            return

        pose_x = pose_y = pose_z = 0.0
        yaw_deg = 0.0
        if self._odom is not None:
            p = self._odom.pose.pose.position
            pose_x, pose_y, pose_z = float(p.x), float(p.y), float(p.z)
            q = self._odom.pose.pose.orientation
            yaw_deg = math.degrees(quat_yaw(q.x, q.y, q.z, q.w))

        payload = bytes(int(max(0, min(100, v))) for v in msg.data)
        self._seq += 1
        meta = build_semantic_token_meta(
            SemanticTokenMetaInput(
                session_id=self._session_id,
                floor_id=self._floor_id,
                tile_id=self._tile_id,
                map_id=self._map_id,
                context_generation=self._context_generation,
                frame_id=str(msg.header.frame_id or "world"),
                grid_width=int(msg.info.width),
                grid_height=int(msg.info.height),
                resolution=float(msg.info.resolution),
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
                origin_z=float(msg.info.origin.position.z),
                pose_x=pose_x,
                pose_y=pose_y,
                pose_z=pose_z,
                pose_yaw_deg=yaw_deg,
                seq=self._seq,
                payload=payload,
            )
        )
        ok = self._client.enqueue_binary(meta, payload)
        if ok and (self._seq == 1 or self._seq % self._log_every_n == 0):
            self.get_logger().info(
                f"Token uplink seq={self._seq} gen={self._context_generation} "
                f"grid={msg.info.width}x{msg.info.height} bytes={len(payload)}"
            )
        event = {
            "event": "token_uplink",
            "seq": self._seq,
            "layer_id": DataLayerId.SPATIAL_VOXEL.value,
            "context_generation": self._context_generation,
            "ok": ok,
        }
        out = String()
        out.data = __import__("json").dumps(event, ensure_ascii=False)
        self._event_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SemanticTokenUplinkNode()
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
