#!/usr/bin/env python3
"""端侧分层数据目录：登记各算法产线视图状态，发布 Catalog，接收层事件."""
from __future__ import annotations

import json
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from data_layer_types import (  # noqa: E402
    CATALOG_SCHEMA,
    DataLayerId,
    LayerCatalog,
    ViewState,
)


class DataLayerRegistryNode(Node):
    def __init__(self) -> None:
        super().__init__("data_layer_registry")
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("tile_id", "main")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("catalog_hz", 1.0)
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("pgo_state_topic", "/lio/backend/pgo_state")
        self.declare_parameter("layer_event_topic", "/edge/data_layer/layer_event")
        self.declare_parameter("mission_event_topic", "/demo/mission/event")
        self.declare_parameter("lifecycle_state_topic", "/demo/lifecycle/state")

        self._catalog = LayerCatalog(
            session_id=str(self.get_parameter("session_id").value),
            floor_id=str(self.get_parameter("floor_id").value),
            tile_id=str(self.get_parameter("tile_id").value),
            map_id=str(self.get_parameter("map_id").value),
        )

        gen_topic = str(self.get_parameter("context_generation_topic").value)
        pgo_topic = str(self.get_parameter("pgo_state_topic").value)
        layer_event_topic = str(self.get_parameter("layer_event_topic").value)
        mission_topic = str(self.get_parameter("mission_event_topic").value)
        lifecycle_topic = str(self.get_parameter("lifecycle_state_topic").value)

        self.create_subscription(UInt64, gen_topic, self._on_context_generation, 10)
        self.create_subscription(String, pgo_topic, self._on_pgo_state, 10)
        self.create_subscription(String, layer_event_topic, self._on_layer_event, 50)
        self.create_subscription(String, mission_topic, self._on_mission_event, 20)
        self.create_subscription(String, lifecycle_topic, self._on_lifecycle_state, 10)

        self._catalog_pub = self.create_publisher(String, "/edge/data_layer/catalog", 10)

        hz = max(0.2, float(self.get_parameter("catalog_hz").value))
        self.create_timer(1.0 / hz, self._publish_catalog)

        self.get_logger().info(
            f"DataLayerRegistry catalog={CATALOG_SCHEMA} "
            f"floor={self._catalog.floor_id} tile={self._catalog.tile_id} @ {hz:.1f}Hz"
        )

    def _publish_catalog(self) -> None:
        msg = String()
        msg.data = json.dumps(self._catalog.snapshot(), ensure_ascii=False)
        self._catalog_pub.publish(msg)

    def _on_context_generation(self, msg: UInt64) -> None:
        gen = int(msg.data)
        if gen > self._catalog.context_generation:
            self.get_logger().info(
                f"context_generation {self._catalog.context_generation} → {gen}, mark stale"
            )
            self._catalog.bump_context_generation(gen)

    def _on_pgo_state(self, msg: String) -> None:
        state = msg.data.strip().upper()
        if state == "OPTIMIZING":
            layer = self._catalog.layers.get(DataLayerId.TOPOLOGY_SUBGRAPH.value)
            if layer is not None and layer.view_state != ViewState.ABSENT.value:
                self._catalog.note_building(DataLayerId.TOPOLOGY_SUBGRAPH.value)

    def _on_layer_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        layer_id = str(body.get("layer_id", ""))
        if not layer_id:
            return
        event = str(body.get("event", "published"))
        if event == "building":
            self._catalog.note_building(layer_id)
            return
        layer_version = int(body.get("layer_version", body.get("version", 0)))
        context_generation = int(
            body.get("context_generation", self._catalog.context_generation)
        )
        error = str(body.get("error", ""))
        self._catalog.note_publish(
            layer_id,
            layer_version=layer_version,
            context_generation=context_generation,
            timestamp_ns=body.get("timestamp_ns"),
            error=error,
        )

    def _emit_layer_event_from_mission(self, body: dict[str, Any]) -> None:
        """mission 事件映射到 exec_feedback 层视图（由 ExecutionFeedbackNode 实际上行）."""
        event = str(body.get("event", ""))
        if not event:
            return
        layer = self._catalog.layers.get(DataLayerId.EXEC_FEEDBACK.value)
        if layer is None:
            return
        if event in ("mission_started", "subgoal_dispatched", "subgoal_reached"):
            layer.view_state = ViewState.READY.value
        elif event in ("plan_fail_hold", "glass_trap", "request_assist"):
            layer.view_state = ViewState.STALE.value
            layer.last_error = event

    def _on_mission_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._emit_layer_event_from_mission(body)

    def _on_lifecycle_state(self, msg: String) -> None:
        state = msg.data.strip().lower()
        if state in ("inactive", "deactivated", "cleaned_up"):
            layer = self._catalog.layers.get(DataLayerId.EXEC_FEEDBACK.value)
            if layer is not None:
                layer.view_state = ViewState.ABSENT.value


def main() -> None:
    rclpy.init()
    node = DataLayerRegistryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
