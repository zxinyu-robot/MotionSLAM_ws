#!/usr/bin/env python3
"""SubgraphPublisher：1Hz + BT 事件触发，上行 sparse_subgraph.v1 JSON."""
from __future__ import annotations

import json
import math
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from data_layer_types import DataLayerId  # noqa: E402
from edge_uplink_client import EdgeTcpUplinkClient  # noqa: E402
from frontier_export import FrontierExportConfig, export_frontiers_from_keyframes  # noqa: E402
from subgraph_snapshot import (  # noqa: E402
    FrontierView,
    KeyframeView,
    SubgraphBuildInput,
    build_sparse_subgraph_v1,
    quat_yaw,
)


BT_TRIGGER_EVENTS = frozenset(
    {
        "replan_timeout",
        "subgoal_retry",
        "plan_fail_hold",
        "plan_fail_recovery_done",
        "pgo_applied_redispatch",
        "subgoal_dispatched",
        "search_frontier_reached",
        "search_exhausted",
    }
)


class SubgraphPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("subgraph_publisher")
        self.declare_parameter("enabled", True)
        self.declare_parameter("edge_host", "127.0.0.1")
        self.declare_parameter("subgraph_port", 9877)
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("tile_id", "main")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("queue_max", 8)
        self.declare_parameter("connect_timeout_s", 2.0)
        self.declare_parameter("send_timeout_s", 3.0)
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("keyframe_poses_topic", "/lio/backend/keyframe_poses")
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("pgo_state_topic", "/lio/backend/pgo_state")
        self.declare_parameter("mission_event_topic", "/demo/mission/event")
        self.declare_parameter("use_bt_subgoal_as_frontier", False)
        self.declare_parameter("frontier_min_dist_m", 1.0)
        self.declare_parameter("frontier_max_dist_m", 12.0)
        self.declare_parameter("frontier_max_count", 8)
        self.declare_parameter("frontier_ray_count", 8)
        self.declare_parameter("frontier_ray_dist_m", 3.0)
        self.declare_parameter("frontier_snapshot_topic", "/demo/mission/frontier_snapshot")
        self.declare_parameter("log_each_uplink", True)
        self.declare_parameter("layer_event_topic", "/edge/data_layer/layer_event")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._session_id = str(self.get_parameter("session_id").value)
        self._floor_id = str(self.get_parameter("floor_id").value)
        self._tile_id = str(self.get_parameter("tile_id").value)
        self._map_id = str(self.get_parameter("map_id").value)
        self._use_bt_frontier = bool(self.get_parameter("use_bt_subgoal_as_frontier").value)
        self._log_each = bool(self.get_parameter("log_each_uplink").value)
        self._frontier_cfg = FrontierExportConfig(
            min_dist_m=float(self.get_parameter("frontier_min_dist_m").value),
            max_dist_m=float(self.get_parameter("frontier_max_dist_m").value),
            max_frontiers=int(self.get_parameter("frontier_max_count").value),
            ray_count=int(self.get_parameter("frontier_ray_count").value),
            ray_dist_m=float(self.get_parameter("frontier_ray_dist_m").value),
        )
        self._last_frontiers: list[FrontierView] = []

        self._version = 0
        self._pending_trigger = "periodic"
        self._odom: Optional[Odometry] = None
        self._keyframes: list[KeyframeView] = []
        self._context_generation = 0
        self._pgo_state = "IDLE"
        self._last_mission_subgoal: Optional[tuple[float, float, float]] = None

        odom_topic = str(self.get_parameter("odom_topic").value)
        kf_topic = str(self.get_parameter("keyframe_poses_topic").value)
        gen_topic = str(self.get_parameter("context_generation_topic").value)
        pgo_topic = str(self.get_parameter("pgo_state_topic").value)
        event_topic = str(self.get_parameter("mission_event_topic").value)

        self._client: Optional[EdgeTcpUplinkClient] = None
        if self._enabled:
            host = str(self.get_parameter("edge_host").value)
            port = int(self.get_parameter("subgraph_port").value)
            self._client = EdgeTcpUplinkClient(
                host=host,
                port=port,
                queue_max=int(self.get_parameter("queue_max").value),
                connect_timeout_s=float(self.get_parameter("connect_timeout_s").value),
                send_timeout_s=float(self.get_parameter("send_timeout_s").value),
                on_error=lambda msg: self.get_logger().warn(msg),
            )

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(PoseArray, kf_topic, self._on_keyframes, 10)
        self.create_subscription(UInt64, gen_topic, self._on_context_generation, 10)
        self.create_subscription(String, pgo_topic, self._on_pgo_state, 10)
        self.create_subscription(String, event_topic, self._on_mission_event, 10)

        hz = max(0.2, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._on_timer)

        self._event_pub = self.create_publisher(String, "/edge/uplink/subgraph_event", 10)
        self._frontier_pub = self.create_publisher(
            String, str(self.get_parameter("frontier_snapshot_topic").value), 10
        )
        self._layer_event_pub = self.create_publisher(
            String, str(self.get_parameter("layer_event_topic").value), 10
        )

        self.get_logger().info(
            f"SubgraphPublisher enabled={self._enabled} "
            f"→ {self.get_parameter('edge_host').value}:"
            f"{self.get_parameter('subgraph_port').value} @ {hz:.1f}Hz"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_keyframes(self, msg: PoseArray) -> None:
        out: list[KeyframeView] = []
        for i, pose in enumerate(msg.poses):
            p = pose.position
            q = pose.orientation
            yaw = quat_yaw(q.x, q.y, q.z, q.w)
            out.append(KeyframeView(i, p.x, p.y, p.z, yaw))
        self._keyframes = out

    def _on_context_generation(self, msg: UInt64) -> None:
        self._context_generation = int(msg.data)

    def _on_pgo_state(self, msg: String) -> None:
        self._pgo_state = msg.data.strip().upper() or "IDLE"

    def _on_mission_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = str(body.get("event", ""))
        if event in BT_TRIGGER_EVENTS:
            self._pending_trigger = "bt_replan"
        if event == "subgoal_dispatched" and self._use_bt_frontier:
            x = body.get("x")
            y = body.get("y")
            z = body.get("z")
            if x is not None and y is not None:
                self._last_mission_subgoal = (float(x), float(y), float(z or 0.35))

    def _build_frontiers(
        self,
        robot_x: float,
        robot_y: float,
        robot_z: float,
        robot_yaw: float,
    ) -> list[FrontierView]:
        if self._use_bt_frontier and self._last_mission_subgoal is not None:
            x, y, z = self._last_mission_subgoal
            return [
                FrontierView(
                    frontier_id="bt_next_subgoal",
                    x=x,
                    y=y,
                    z=z if z > 0 else robot_z,
                    geom_score=1.0,
                )
            ]
        return export_frontiers_from_keyframes(
            robot_x,
            robot_y,
            robot_z,
            robot_yaw,
            self._keyframes,
            config=self._frontier_cfg,
        )

    def _publish_frontier_snapshot(self, frontiers: list[FrontierView], version: int) -> None:
        body = {
            "version": version,
            "frontiers": [
                {
                    "frontier_id": f.frontier_id,
                    "x": f.x,
                    "y": f.y,
                    "z": f.z,
                    "geom_score": f.geom_score,
                }
                for f in frontiers
            ],
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._frontier_pub.publish(msg)

    def _snapshot(self, trigger_reason: str) -> Optional[dict[str, Any]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = quat_yaw(q.x, q.y, q.z, q.w)
        frontiers = self._build_frontiers(p.x, p.y, max(p.z, 0.3), yaw)
        self._last_frontiers = frontiers
        self._version += 1
        payload = build_sparse_subgraph_v1(
            SubgraphBuildInput(
                version=self._version,
                session_id=self._session_id,
                floor_id=self._floor_id,
                tile_id=self._tile_id,
                map_id=self._map_id,
                trigger_reason=trigger_reason,
                robot_x=p.x,
                robot_y=p.y,
                robot_z=p.z,
                robot_yaw=yaw,
                keyframes=self._keyframes,
                frontiers=frontiers,
                context_generation=self._context_generation,
                pgo_state=self._pgo_state,
            )
        )
        self._publish_frontier_snapshot(frontiers, self._version)
        return payload

    def _publish_event(self, event: str, **extra: Any) -> None:
        body = {"event": event, "version": self._version, **extra}
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._event_pub.publish(msg)

    def _publish_layer_event(self, event: str, **extra: Any) -> None:
        body = {
            "layer_id": DataLayerId.TOPOLOGY_SUBGRAPH.value,
            "event": event,
            "layer_version": self._version,
            "context_generation": self._context_generation,
            **extra,
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._layer_event_pub.publish(msg)

    def _on_timer(self) -> None:
        if not self._enabled or self._client is None:
            return
        trigger = self._pending_trigger
        self._pending_trigger = "periodic"
        snap = self._snapshot(trigger)
        if snap is None:
            return
        ok = self._client.enqueue(snap)
        if self._log_each:
            self.get_logger().info(
                f"SUBGRAPH_UPLINK enqueue v={snap['version']} "
                f"nodes={len(snap['nodes'])} frontiers={len(snap['frontiers'])} "
                f"trigger={trigger} ok={ok}"
            )
        self._publish_event(
            "SUBGRAPH_UPLINK_OK" if ok else "SUBGRAPH_UPLINK_FAIL",
            trigger=trigger,
            nodes=len(snap["nodes"]),
            frontiers=len(snap["frontiers"]),
        )
        self._publish_layer_event(
            "published" if ok else "publish_failed",
            error="" if ok else "uplink_queue_full",
        )

    def destroy_node(self) -> None:
        if self._client is not None:
            self._client.stop()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SubgraphPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
