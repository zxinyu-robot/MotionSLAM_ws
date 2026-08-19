#!/usr/bin/env python3
"""MVPI1 CommandExecutor：ActionGroup 校验、任务事件发布、拒绝时安全停车."""
from __future__ import annotations

import json
import math
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from action_group import (  # noqa: E402
    ActionGroupValidation,
    extract_nav_waypoint,
    extract_primary_stage_type,
    has_navigate_stage,
    has_search_stage,
    validate_action_group,
)
from behavior_mode import (  # noqa: E402
    SCHEMA as THING_ENVELOPE_SCHEMA,
    extract_nav_goal,
    extract_skills,
    validate_thing_envelope,
)
from task_context import TaskIdentityContext  # noqa: E402

try:
    from motionslam_msgs.msg import MotionCommand
except ImportError:
    MotionCommand = None  # type: ignore[misc, assignment]


class CommandExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("command_executor")
        self.declare_parameter("enabled", True)
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("local_map_version", 0)
        self.declare_parameter("require_known_frontier", False)
        self.declare_parameter("known_frontier_ids", ["bt_next_subgoal"])
        self.declare_parameter("require_lifecycle", True)
        self.declare_parameter("max_odom_age_s", 2.0)
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("pgo_state_topic", "/lio/backend/pgo_state")
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("subgraph_event_topic", "/edge/uplink/subgraph_event")
        self.declare_parameter("publish_nav_goal_on_accept", True)
        self.declare_parameter("require_behavior_mode", False)
        self.declare_parameter("robot_id", "go2_001")
        self.declare_parameter("platform", "unitree_go2_edu")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._session_id = str(self.get_parameter("session_id").value)
        self._floor_id = str(self.get_parameter("floor_id").value)
        self._map_id = str(self.get_parameter("map_id").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._local_map_version = int(self.get_parameter("local_map_version").value)
        self._require_known = bool(self.get_parameter("require_known_frontier").value)
        self._require_lifecycle = bool(self.get_parameter("require_lifecycle").value)
        self._max_odom_age_s = float(self.get_parameter("max_odom_age_s").value)
        self._publish_goal = bool(self.get_parameter("publish_nav_goal_on_accept").value)
        self._require_behavior_mode = bool(self.get_parameter("require_behavior_mode").value)
        self._robot_id = str(self.get_parameter("robot_id").value)
        self._platform = str(self.get_parameter("platform").value)

        ids = self.get_parameter("known_frontier_ids").value
        self._known_frontiers = {str(x) for x in ids} if isinstance(ids, (list, tuple)) else set()

        self._odom: Optional[Odometry] = None
        self._odom_rx_at = 0.0
        self._lifecycle = "unconfigured"
        self._nav_ready = False
        self._pgo_state = "IDLE"
        self._context_generation = 0
        self._active_command_id = ""

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        odom_topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(String, "/demo/lifecycle/state", self._on_lifecycle, qos)
        self.create_subscription(Bool, "navigation_context/ready", self._on_nav_ready, qos)
        self.create_subscription(String, str(self.get_parameter("pgo_state_topic").value), self._on_pgo, 10)
        self.create_subscription(
            UInt64, str(self.get_parameter("context_generation_topic").value), self._on_context_gen, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("subgraph_event_topic").value), self._on_subgraph_event, 10
        )
        self.create_subscription(String, "/semantic/action_group", self._on_action_group, 10)

        self._event_pub = self.create_publisher(String, "/demo/mission/event", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/demo/mission/semantic_goal", 10)
        if MotionCommand is not None:
            self._motion_pub = self.create_publisher(MotionCommand, "/motion/command", 10)
        else:
            self._motion_pub = None

        self.get_logger().info(
            f"CommandExecutor enabled={self._enabled} "
            f"session={self._session_id} floor={self._floor_id} map={self._map_id}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_rx_at = time.time()

    def _on_lifecycle(self, msg: String) -> None:
        self._lifecycle = msg.data.strip().lower()

    def _on_nav_ready(self, msg: Bool) -> None:
        self._nav_ready = bool(msg.data)

    def _on_pgo(self, msg: String) -> None:
        self._pgo_state = msg.data.strip().upper() or "IDLE"

    def _on_context_gen(self, msg: UInt64) -> None:
        self._context_generation = int(msg.data)

    def _on_subgraph_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ver = body.get("version")
        if isinstance(ver, int):
            self._local_map_version = max(self._local_map_version, ver)
        frontiers = body.get("frontiers") or []
        if not isinstance(frontiers, list):
            return
        for item in frontiers:
            if isinstance(item, dict) and item.get("frontier_id"):
                self._known_frontiers.add(str(item["frontier_id"]))

    def _build_context(self) -> TaskIdentityContext:
        odom_age = time.time() - self._odom_rx_at if self._odom_rx_at > 0 else 999.0
        return TaskIdentityContext(
            local_session_id=self._session_id,
            local_floor_id=self._floor_id,
            local_map_id=self._map_id,
            local_frame_id=self._frame_id,
            local_map_version=self._local_map_version,
            context_generation=self._context_generation,
            lifecycle_active=self._lifecycle == "active",
            localization_ok=self._nav_ready and self._odom is not None,
            pgo_state=self._pgo_state,
            odom_age_s=odom_age,
            max_odom_age_s=self._max_odom_age_s,
            known_frontier_ids=set(self._known_frontiers),
            require_known_frontier=self._require_known,
            local_robot_id=self._robot_id,
            local_platform=self._platform,
        )

    def _emit_event(self, event: str, **payload: Any) -> None:
        body = {
            "event": event,
            "source": "command_executor",
            "command_id": payload.pop("command_id", self._active_command_id),
            "session_id": self._session_id,
            "floor_id": self._floor_id,
            "map_id": self._map_id,
            "lifecycle": self._lifecycle,
            "navigation_ready": self._nav_ready,
            **payload,
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._event_pub.publish(msg)

    def _safe_stop(self) -> None:
        if self._motion_pub is None or MotionCommand is None:
            return
        zero = MotionCommand()
        zero.header.frame_id = "body"
        for _ in range(3):
            zero.header.stamp = self.get_clock().now().to_msg()
            self._motion_pub.publish(zero)

    def _publish_nav_goal_xyz(self, wp: dict[str, Any], frame_id: str) -> None:
        if not self._publish_goal:
            return
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(wp["x"])
        goal.pose.position.y = float(wp["y"])
        goal.pose.position.z = float(wp.get("z", 0.35))
        yaw = float(wp.get("yaw_rad", 0.0))
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        self._goal_pub.publish(goal)

    def _publish_nav_goal(self, action_group: dict[str, Any]) -> None:
        wp = extract_nav_waypoint(action_group)
        if wp is None:
            return
        self._publish_nav_goal_xyz(wp, str(action_group.get("frame_id", self._frame_id)))

    def _handle_validation(self, result: ActionGroupValidation) -> None:
        if not result.ok:
            self._safe_stop()
            self._emit_event(
                "task_rejected",
                accepted=False,
                reason=result.reason,
                safety_state="HOLD",
            )
            self.get_logger().warn(f"ActionGroup REJECT: {result.reason}")
            return

        assert result.action_group is not None
        ag = result.action_group
        self._active_command_id = str(ag.get("command_id", ""))
        stage_ids = [s.get("stage_id") for s in ag.get("stages") or []]
        self._emit_event(
            "task_accepted",
            accepted=True,
            reason="ACCEPT",
            task_type=str(ag.get("task_type", "OBJNAV")),
            language_query=str(ag.get("language_query", "")),
            stage_ids=stage_ids,
            safety_state="OK",
        )
        self.get_logger().info(
            f"ActionGroup ACCEPT command_id={self._active_command_id} stages={stage_ids}"
        )
        self._dispatch_accepted_action_group(ag)

    def _dispatch_accepted_action_group(self, ag: dict[str, Any]) -> None:
        primary = extract_primary_stage_type(ag)
        if primary == "SEARCH" or (has_search_stage(ag) and not has_navigate_stage(ag)):
            self._emit_event(
                "search_explore",
                accepted=True,
                phase="SEARCH",
                reason="SEARCH_ARMED",
            )
            self.get_logger().info(
                f"ActionGroup SEARCH armed command_id={self._active_command_id}"
            )
            return
        if has_navigate_stage(ag):
            self._emit_event(
                "search_navigate",
                accepted=True,
                phase="NAV",
                reason="NAV_ARMED",
            )
            self._publish_nav_goal(ag)
            return
        self.get_logger().warn(
            f"ActionGroup ACCEPT without NAV/SEARCH dispatch command_id={self._active_command_id}"
        )

    def _on_action_group(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            raw = json.loads(msg.data)
        except json.JSONDecodeError:
            self._emit_event("task_rejected", accepted=False, reason="REJECT_JSON")
            self._safe_stop()
            return

        ctx = self._build_context()
        if self._require_lifecycle and not ctx.lifecycle_active:
            self._handle_validation(ActionGroupValidation(False, "REJECT_LIFECYCLE"))
            return

        schema = str(raw.get("schema", ""))
        if schema == THING_ENVELOPE_SCHEMA:
            result = validate_thing_envelope(raw, ctx, require_behavior_mode=True)
            self._handle_thing_envelope(result)
            return

        if self._require_behavior_mode:
            self._handle_validation(ActionGroupValidation(False, "REJECT_MISSING_BEHAVIOR_MODE"))
            return

        result = validate_action_group(raw, ctx)
        self._handle_validation(result)

    def _handle_thing_envelope(self, result: Any) -> None:
        if not result.ok:
            self._safe_stop()
            self._emit_event(
                "task_rejected",
                accepted=False,
                reason=result.reason,
                safety_state="HOLD",
                mode_state="TERMINATED",
            )
            self.get_logger().warn(f"ThingEnvelope REJECT: {result.reason}")
            return

        env = result.envelope
        assert env is not None
        data = env.get("data") if isinstance(env.get("data"), dict) else {}
        props = data.get("properties") if isinstance(data.get("properties"), dict) else {}
        policy = data.get("policy") if isinstance(data.get("policy"), dict) else {}
        self._active_command_id = str(env.get("tid") or env.get("command_id") or "")
        skills = extract_skills(env)
        self._emit_event(
            "task_accepted",
            accepted=True,
            reason="ACCEPT",
            mode_id=str(policy.get("mode_id", "")),
            mode_state="ARMED",
            scene_type=str(props.get("scene_type", "")),
            skills=skills,
            safety_state="OK",
        )
        self.get_logger().info(
            f"ThingEnvelope ACCEPT tid={self._active_command_id} "
            f"mode={policy.get('mode_id')} skills={skills}"
        )
        if "Navigate" in skills:
            goal = extract_nav_goal(env)
            if goal is None:
                self._safe_stop()
                self._emit_event(
                    "task_rejected",
                    accepted=False,
                    reason="REJECT_UNRESOLVED_GOAL",
                    safety_state="HOLD",
                    mode_state="TERMINATED",
                )
                return
            self._emit_event(
                "search_navigate",
                accepted=True,
                phase="NAV",
                reason="NAV_ARMED",
                mode_id=str(policy.get("mode_id", "")),
                mode_state="ACTIVE",
            )
            self._publish_nav_goal_xyz(
                goal, str(props.get("frame_id", self._frame_id))
            )
            return
        if "ExploreFrontier" in skills:
            self._emit_event(
                "search_explore",
                accepted=True,
                phase="SEARCH",
                reason="SEARCH_ARMED",
                mode_id=str(policy.get("mode_id", "")),
                mode_state="ACTIVE",
            )


def main() -> None:
    rclpy.init()
    node = CommandExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
