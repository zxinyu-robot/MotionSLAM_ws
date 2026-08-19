#!/usr/bin/env python3
"""SemanticDirective / ActionGroup 下行：TCP listen :9879 → ROS 话题."""
from __future__ import annotations

import json
import queue
import socket
import threading
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from behavior_mode import SCHEMA as THING_ENVELOPE_SCHEMA  # noqa: E402
from behavior_mode import validate_thing_envelope  # noqa: E402
from semantic_directive import SCHEMA as DIRECTIVE_SCHEMA  # noqa: E402
from semantic_directive import validate_semantic_directive  # noqa: E402
from task_context import TaskIdentityContext  # noqa: E402

ACTION_GROUP_SCHEMA = "motionslam.action_group.v1"


class DirectiveReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("directive_receiver")
        self.declare_parameter("enabled", True)
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 9879)
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("local_map_version", 1_000_000)
        self.declare_parameter("require_known_frontier", False)
        self.declare_parameter("known_frontier_ids", ["bt_next_subgoal"])
        self.declare_parameter("subgraph_event_topic", "/edge/uplink/subgraph_event")
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("require_behavior_mode", False)
        self.declare_parameter("robot_id", "go2_001")
        self.declare_parameter("platform", "unitree_go2_edu")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._listen_host = str(self.get_parameter("listen_host").value)
        self._listen_port = int(self.get_parameter("listen_port").value)
        self._session_id = str(self.get_parameter("session_id").value)
        self._floor_id = str(self.get_parameter("floor_id").value)
        self._map_id = str(self.get_parameter("map_id").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._local_map_version = int(self.get_parameter("local_map_version").value)
        self._require_known = bool(self.get_parameter("require_known_frontier").value)
        self._require_behavior_mode = bool(self.get_parameter("require_behavior_mode").value)
        self._robot_id = str(self.get_parameter("robot_id").value)
        self._platform = str(self.get_parameter("platform").value)
        self._context_generation = 0
        ids = self.get_parameter("known_frontier_ids").value
        self._known_frontiers = {str(x) for x in ids} if isinstance(ids, (list, tuple)) else set()

        self._inbox: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=32)
        self._stop = threading.Event()
        self._server_thread: Optional[threading.Thread] = None

        self._directive_pub = self.create_publisher(String, "/semantic/directive", 10)
        self._action_group_pub = self.create_publisher(String, "/semantic/action_group", 10)
        self._valid_pub = self.create_publisher(Bool, "/semantic/valid", 10)
        self._event_pub = self.create_publisher(String, "/edge/uplink/directive_event", 10)

        event_topic = str(self.get_parameter("subgraph_event_topic").value)
        gen_topic = str(self.get_parameter("context_generation_topic").value)
        self.create_subscription(String, event_topic, self._on_subgraph_event, 10)
        self.create_subscription(UInt64, gen_topic, self._on_context_generation, 10)
        self.create_timer(0.2, self._process_inbox)

        if self._enabled:
            self._server_thread = threading.Thread(
                target=self._serve_tcp, name="directive-receiver", daemon=True
            )
            self._server_thread.start()

        self.get_logger().info(
            f"DirectiveReceiver enabled={self._enabled} "
            f"listen {self._listen_host}:{self._listen_port}"
        )

    def _on_subgraph_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ver = body.get("version")
        if isinstance(ver, int):
            self._local_map_version = max(self._local_map_version, ver)

    def _on_context_generation(self, msg: UInt64) -> None:
        self._context_generation = int(msg.data)

    def _build_context(self) -> TaskIdentityContext:
        return TaskIdentityContext(
            local_session_id=self._session_id,
            local_floor_id=self._floor_id,
            local_map_id=self._map_id,
            local_frame_id=self._frame_id,
            local_map_version=self._local_map_version,
            context_generation=self._context_generation,
            lifecycle_active=True,
            localization_ok=True,
            known_frontier_ids=set(self._known_frontiers),
            require_known_frontier=self._require_known,
            local_robot_id=self._robot_id,
            local_platform=self._platform,
        )

    def _serve_tcp(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self._listen_host, self._listen_port))
            srv.listen(4)
            srv.settimeout(1.0)
        except OSError as exc:
            self.get_logger().error(f"DirectiveReceiver bind failed: {exc}")
            return

        self.get_logger().info(
            f"DirectiveReceiver TCP ready on {self._listen_host}:{self._listen_port}"
        )
        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(
                target=self._handle_client,
                args=(conn, addr),
                daemon=True,
            ).start()
        srv.close()

    def _handle_client(self, conn: socket.socket, addr: Any) -> None:
        buf = b""
        try:
            conn.settimeout(2.0)
            while not self._stop.is_set():
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        obj = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError:
                        self.get_logger().warn(f"directive JSON parse fail from {addr}")
                        continue
                    try:
                        self._inbox.put_nowait(obj)
                    except queue.Full:
                        self.get_logger().warn("directive inbox full, drop")
        finally:
            conn.close()

    def _publish_event(self, event: str, reason: str, schema: str = "") -> None:
        em = String()
        em.data = json.dumps(
            {"event": event, "reason": reason, "schema": schema},
            ensure_ascii=False,
        )
        self._event_pub.publish(em)

    def _process_inbox(self) -> None:
        while True:
            try:
                raw = self._inbox.get_nowait()
            except queue.Empty:
                break

            schema = str(raw.get("schema", ""))
            if schema == THING_ENVELOPE_SCHEMA:
                ctx = self._build_context()
                result = validate_thing_envelope(
                    raw, ctx, require_behavior_mode=True
                )
                if result.ok and result.envelope is not None:
                    ag_msg = String()
                    ag_msg.data = json.dumps(result.envelope, ensure_ascii=False)
                    self._action_group_pub.publish(ag_msg)
                    vmsg = Bool()
                    vmsg.data = True
                    self._valid_pub.publish(vmsg)
                    self._publish_event("THING_ENVELOPE_ACCEPT", result.reason, schema)
                    self.get_logger().info(
                        f"ThingEnvelope ACCEPT tid={raw.get('tid')} method={raw.get('method')}"
                    )
                else:
                    vmsg = Bool()
                    vmsg.data = False
                    self._valid_pub.publish(vmsg)
                    self._publish_event("THING_ENVELOPE_REJECT", result.reason, schema)
                    self.get_logger().warn(result.reason)
                continue

            if schema == ACTION_GROUP_SCHEMA:
                if self._require_behavior_mode:
                    vmsg = Bool()
                    vmsg.data = False
                    self._valid_pub.publish(vmsg)
                    self._publish_event(
                        "ACTION_GROUP_REJECT",
                        "REJECT_MISSING_BEHAVIOR_MODE",
                        schema,
                    )
                    self.get_logger().warn("REJECT_MISSING_BEHAVIOR_MODE")
                    continue
                ag_msg = String()
                ag_msg.data = json.dumps(raw, ensure_ascii=False)
                self._action_group_pub.publish(ag_msg)
                self._publish_event("ACTION_GROUP_RX", "FORWARD", schema)
                self.get_logger().info(
                    f"ActionGroup forward command_id={raw.get('command_id')}"
                )
                continue

            ctx = self._build_context()
            result = validate_semantic_directive(raw, ctx)
            if result.ok and result.directive is not None:
                dmsg = String()
                dmsg.data = json.dumps(result.directive, ensure_ascii=False)
                self._directive_pub.publish(dmsg)
                vmsg = Bool()
                vmsg.data = True
                self._valid_pub.publish(vmsg)
                self._publish_event("DIRECTIVE_ACCEPT", result.reason, DIRECTIVE_SCHEMA)
                self.get_logger().info(
                    f"DIRECTIVE_ACCEPT query={result.directive.get('language_query')}"
                )
            else:
                vmsg = Bool()
                vmsg.data = False
                self._valid_pub.publish(vmsg)
                evt = "DIRECTIVE_REJECT"
                self._publish_event(evt, result.reason, schema or DIRECTIVE_SCHEMA)
                self.get_logger().warn(result.reason)

    def destroy_node(self) -> None:
        self._stop.set()
        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DirectiveReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
