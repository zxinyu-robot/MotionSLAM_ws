#!/usr/bin/env python3
"""任务 B：订阅 mission/lifecycle 事件，分层上行 execution_feedback 至边端."""
from __future__ import annotations

import json
import math
import time
from typing import Any, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from data_layer_types import DataLayerId  # noqa: E402
from edge_uplink_client import EdgeTcpUplinkClient  # noqa: E402
from execution_feedback import ExecutionFeedbackInput, build_execution_feedback_v1  # noqa: E402
from subgraph_snapshot import quat_yaw  # noqa: E402


FEEDBACK_EVENTS = frozenset(
    {
        "mission_started",
        "subgoal_dispatched",
        "subgoal_reached",
        "subgoal_retry",
        "alignment_timeout",
        "replan_fuse",
        "plan_fail_hold",
        "plan_fail_recovery_done",
        "pgo_applied_redispatch",
        "glass_trap",
        "return_home_started",
        "return_home_done",
        "request_assist",
        "mission_cancelled",
        "task_accepted",
        "task_rejected",
        "search_explore",
        "semantic_goal_armed",
        "search_navigate",
        "search_mission_armed",
        "search_frontier_reached",
        "frontier_selected",
        "search_handoff",
        "object_found",
        "search_exhausted",
        "search_frontier_rotated",
        "search_plan_fail",
        "search_persist_reset",
        "task_canceled",
    }
)


class ExecutionFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("execution_feedback")
        self.declare_parameter("enabled", True)
        self.declare_parameter("edge_host", "127.0.0.1")
        self.declare_parameter("feedback_port", 9880)
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("tile_id", "main")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("queue_max", 16)
        self.declare_parameter("connect_timeout_s", 2.0)
        self.declare_parameter("send_timeout_s", 3.0)
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("mission_event_topic", "/demo/mission/event")
        self.declare_parameter("lifecycle_state_topic", "/demo/lifecycle/state")
        self.declare_parameter("layer_event_topic", "/edge/data_layer/layer_event")
        self.declare_parameter("log_each_uplink", True)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._session_id = str(self.get_parameter("session_id").value)
        self._floor_id = str(self.get_parameter("floor_id").value)
        self._tile_id = str(self.get_parameter("tile_id").value)
        self._map_id = str(self.get_parameter("map_id").value)
        self._log_each = bool(self.get_parameter("log_each_uplink").value)
        self._layer_version = 0
        self._context_generation = 0
        self._odom: Optional[Odometry] = None

        self._client: Optional[EdgeTcpUplinkClient] = None
        if self._enabled:
            self._client = EdgeTcpUplinkClient(
                host=str(self.get_parameter("edge_host").value),
                port=int(self.get_parameter("feedback_port").value),
                queue_max=int(self.get_parameter("queue_max").value),
                connect_timeout_s=float(self.get_parameter("connect_timeout_s").value),
                send_timeout_s=float(self.get_parameter("send_timeout_s").value),
                on_error=lambda msg: self.get_logger().warn(msg),
            )

        odom_topic = str(self.get_parameter("odom_topic").value)
        gen_topic = str(self.get_parameter("context_generation_topic").value)
        mission_topic = str(self.get_parameter("mission_event_topic").value)
        lifecycle_topic = str(self.get_parameter("lifecycle_state_topic").value)

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(UInt64, gen_topic, self._on_context_generation, 10)
        self.create_subscription(String, mission_topic, self._on_mission_event, 20)
        self.create_subscription(String, lifecycle_topic, self._on_lifecycle_state, 10)

        self._layer_event_pub = self.create_publisher(
            String, str(self.get_parameter("layer_event_topic").value), 10
        )
        self._uplink_ok_pub = self.create_publisher(Bool, "/demo/mission/uplink_ok", 10)
        self.create_timer(1.0, self._tick_uplink_ok)

        self.get_logger().info(
            f"ExecutionFeedback enabled={self._enabled} "
            f"→ {self.get_parameter('edge_host').value}:"
            f"{self.get_parameter('feedback_port').value}"
        )

    def _tick_uplink_ok(self) -> None:
        ok = False
        if self._enabled and self._client is not None:
            self._client.enqueue(
                {
                    "schema": "motionslam.uplink_heartbeat.v1",
                    "event": "heartbeat",
                }
            )
            last = float(self._client.stats.last_ok_mono)
            ok = last > 0.0 and (time.monotonic() - last) < 8.0
        msg = Bool()
        msg.data = ok
        self._uplink_ok_pub.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_context_generation(self, msg: UInt64) -> None:
        self._context_generation = int(msg.data)

    def _robot_pose(self) -> tuple[float, float, float, float]:
        if self._odom is None:
            return 0.0, 0.0, 0.0, 0.0
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = quat_yaw(q.x, q.y, q.z, q.w)
        return p.x, p.y, p.z, math.degrees(yaw)

    def _publish_layer_event(self, event: str, **extra: Any) -> None:
        body = {
            "layer_id": DataLayerId.EXEC_FEEDBACK.value,
            "event": event,
            "layer_version": self._layer_version,
            "context_generation": self._context_generation,
            **extra,
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._layer_event_pub.publish(msg)

    def _uplink_feedback(
        self,
        *,
        event: str,
        phase: str = "",
        accepted: bool = True,
        reason: str = "",
        safety_state: str = "OK",
        extra: Optional[dict[str, Any]] = None,
    ) -> None:
        self._layer_version += 1
        x, y, z, yaw_deg = self._robot_pose()
        extra_body = dict(extra or {})
        mode_id = str(extra_body.pop("mode_id", "") or "")
        mode_state = str(extra_body.pop("mode_state", "") or "")
        scene_type = str(extra_body.pop("scene_type", "") or "")
        events = extra_body.pop("events", None)
        world_fragments = extra_body.pop("world_fragments", None)
        payload = build_execution_feedback_v1(
            ExecutionFeedbackInput(
                session_id=self._session_id,
                floor_id=self._floor_id,
                tile_id=self._tile_id,
                map_id=self._map_id,
                context_generation=self._context_generation,
                layer_version=self._layer_version,
                phase=phase,
                event=event,
                accepted=accepted,
                reason=reason,
                safety_state=safety_state,
                robot_x=x,
                robot_y=y,
                robot_z=z,
                robot_yaw_deg=yaw_deg,
                extra=extra_body or None,
                mode_id=mode_id,
                mode_state=mode_state,
                scene_type=scene_type,
                events=events if isinstance(events, list) else None,
                world_fragments=world_fragments if isinstance(world_fragments, list) else None,
            )
        )
        ok = False
        if self._enabled and self._client is not None:
            ok = self._client.enqueue(payload)
        self._publish_layer_event(
            "published" if ok else "publish_failed",
            error="" if ok else "uplink_queue_full_or_disabled",
        )
        if self._log_each:
            self.get_logger().info(
                f"EXEC_FEEDBACK event={event} v={self._layer_version} ok={ok}"
            )

    def _on_mission_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = str(body.get("event", ""))
        if event not in FEEDBACK_EVENTS:
            return
        safety = "OK"
        accepted = True
        reason = ""
        if event in ("plan_fail_hold", "glass_trap", "request_assist"):
            safety = "DEGRADED"
            reason = event
        elif event == "task_rejected":
            safety = "HOLD"
            accepted = False
            reason = str(body.get("reason", "task_rejected"))
        phase = str(body.get("phase", ""))
        self._uplink_feedback(
            event=event,
            phase=phase,
            accepted=accepted,
            reason=reason,
            safety_state=safety,
            extra={k: v for k, v in body.items() if k not in ("event", "phase")},
        )

    def _on_lifecycle_state(self, msg: String) -> None:
        state = msg.data.strip().lower()
        if state not in ("active", "inactive", "deactivated"):
            return
        self._uplink_feedback(
            event=f"lifecycle_{state}",
            phase="lifecycle",
            accepted=state == "active",
            safety_state="OK" if state == "active" else "HOLD",
        )

    def destroy_node(self) -> None:
        if self._client is not None:
            self._client.stop()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ExecutionFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
