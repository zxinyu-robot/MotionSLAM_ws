#!/usr/bin/env python3
"""RGB 上行：/frontvideostream DDS 载荷原样 TCP 转发."""
from __future__ import annotations

import json
import math
from typing import Any, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from data_layer_types import DataLayerId  # noqa: E402
from edge_uplink_client import EdgeTcpUplinkClient  # noqa: E402
from rgb_forward_frame import (  # noqa: E402
    RgbForwardMetaInput,
    build_rgb_forward_meta,
    pack_rgb_forward_frame,
)
from subgraph_snapshot import quat_yaw  # noqa: E402

try:
    from unitree_go.msg import Go2FrontVideoData
except ImportError:
    Go2FrontVideoData = None  # type: ignore[misc, assignment]

STREAM_FIELDS = ("video360p", "video180p", "video720p")


class RgbKeyframeUplinkNode(Node):
    def __init__(self) -> None:
        super().__init__("rgb_keyframe_uplink")
        self.declare_parameter("enabled", True)
        self.declare_parameter("edge_host", "127.0.0.1")
        self.declare_parameter("rgb_port", 9878)
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("tile_id", "main")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("stream_field", "video360p")
        self.declare_parameter("video_topic", "/frontvideostream")
        self.declare_parameter("queue_max", 4)
        self.declare_parameter("connect_timeout_s", 2.0)
        self.declare_parameter("send_timeout_s", 8.0)
        self.declare_parameter("min_payload_bytes", 32)
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("subgraph_event_topic", "/edge/uplink/subgraph_event")
        self.declare_parameter("layer_event_topic", "/edge/data_layer/layer_event")
        self.declare_parameter("log_each_uplink", False)
        self.declare_parameter("log_every_n", 30)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._session_id = str(self.get_parameter("session_id").value)
        self._floor_id = str(self.get_parameter("floor_id").value)
        self._tile_id = str(self.get_parameter("tile_id").value)
        self._map_id = str(self.get_parameter("map_id").value)
        self._stream_field = str(self.get_parameter("stream_field").value)
        self._video_topic = str(self.get_parameter("video_topic").value)
        self._min_payload = int(self.get_parameter("min_payload_bytes").value)
        self._log_each = bool(self.get_parameter("log_each_uplink").value)
        self._log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self._odom: Optional[Odometry] = None
        self._context_generation = 0
        self._subgraph_version = 0
        self._seq = 0

        self._client: Optional[EdgeTcpUplinkClient] = None
        if self._enabled:
            self._client = EdgeTcpUplinkClient(
                host=str(self.get_parameter("edge_host").value),
                port=int(self.get_parameter("rgb_port").value),
                queue_max=int(self.get_parameter("queue_max").value),
                connect_timeout_s=float(self.get_parameter("connect_timeout_s").value),
                send_timeout_s=float(self.get_parameter("send_timeout_s").value),
                binary_packer=pack_rgb_forward_frame,
                on_error=lambda msg: self.get_logger().warn(msg),
            )

        odom_topic = str(self.get_parameter("odom_topic").value)
        gen_topic = str(self.get_parameter("context_generation_topic").value)
        event_topic = str(self.get_parameter("subgraph_event_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(UInt64, gen_topic, self._on_context_generation, 10)
        self.create_subscription(String, event_topic, self._on_subgraph_event, 10)

        if Go2FrontVideoData is None:
            self.get_logger().error("unitree_go 不可用，无法订阅 /frontvideostream")
        else:
            self.create_subscription(
                Go2FrontVideoData, self._video_topic, self._on_front_video, 10
            )

        self._event_pub = self.create_publisher(String, "/edge/uplink/rgb_event", 10)
        self._layer_event_pub = self.create_publisher(
            String, str(self.get_parameter("layer_event_topic").value), 10
        )
        self.get_logger().info(
            f"RgbForward enabled={self._enabled} topic={self._video_topic} "
            f"field={self._stream_field} → "
            f"{self.get_parameter('edge_host').value}:{self.get_parameter('rgb_port').value}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_context_generation(self, msg: UInt64) -> None:
        self._context_generation = int(msg.data)

    def _on_subgraph_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ver = body.get("version")
        if isinstance(ver, int):
            self._subgraph_version = ver

    def _pick_stream_bytes(self, msg: Any) -> tuple[Optional[bytes], str]:
        order = [self._stream_field] + [f for f in STREAM_FIELDS if f != self._stream_field]
        for field in order:
            data = getattr(msg, field, None)
            if data:
                return bytes(data), field
        return None, self._stream_field

    def _pose_fields(self) -> tuple[float, float, float, float]:
        if self._odom is None:
            return 0.0, 0.0, 0.0, 0.0
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = quat_yaw(q.x, q.y, q.z, q.w)
        return p.x, p.y, p.z, math.degrees(yaw)

    def _on_front_video(self, msg: Any) -> None:
        if not self._enabled or self._client is None:
            return
        payload, field = self._pick_stream_bytes(msg)
        if not payload or len(payload) < self._min_payload:
            return

        self._seq += 1
        px, py, pz, yaw_deg = self._pose_fields()
        meta = build_rgb_forward_meta(
            RgbForwardMetaInput(
                session_id=self._session_id,
                stream_field=field,
                payload=payload,
                time_frame=int(msg.time_frame),
                seq=self._seq,
                floor_id=self._floor_id,
                tile_id=self._tile_id,
                map_id=self._map_id,
                subgraph_version=self._subgraph_version,
                context_generation=self._context_generation,
                pose_x=px,
                pose_y=py,
                pose_z=pz,
                pose_yaw_deg=yaw_deg,
                topic=self._video_topic,
            )
        )
        ok = self._client.enqueue_binary(meta, payload)
        if ok and self._seq % self._log_every_n == 0:
            layer_evt = String()
            layer_evt.data = json.dumps(
                {
                    "layer_id": DataLayerId.PERCEPTION_RGB.value,
                    "event": "published",
                    "layer_version": self._seq,
                    "context_generation": self._context_generation,
                },
                ensure_ascii=False,
            )
            self._layer_event_pub.publish(layer_evt)
        if self._log_each or (self._seq % self._log_every_n == 0):
            self.get_logger().info(
                f"RGB_FORWARD seq={self._seq} field={field} bytes={len(payload)} "
                f"time_frame={meta['time_frame']} ok={ok}"
            )
        if self._seq % self._log_every_n == 0:
            evt = String()
            evt.data = json.dumps(
                {
                    "event": "RGB_FORWARD_OK" if ok else "RGB_FORWARD_FAIL",
                    "seq": self._seq,
                    "stream_field": field,
                    "bytes": len(payload),
                    "time_frame": meta["time_frame"],
                },
                ensure_ascii=False,
            )
            self._event_pub.publish(evt)

    def destroy_node(self) -> None:
        if self._client is not None:
            self._client.stop()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RgbKeyframeUplinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
