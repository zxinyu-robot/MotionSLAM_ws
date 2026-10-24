#!/usr/bin/env python3
"""Thin Harness：目标架构 Task Intent API（:9879 下行）+ Execution Feedback API（:9880 上行）。

世界空间（边端/云端）↔ 符号空间（本节点）↔ 度量空间（BT/PCT/SCAN/执行器）。
只下发技能级意图，不直发速度；执行权在设备侧。

接入方式：BT 以 semantic_objnav 模式运行；harness 把 Navigate 技能的 goal
翻译成 /demo/mission/semantic_goal，ReturnHome 翻译成 /demo/mission/return_home；
订阅 /demo/mission/event 转成 Execution Feedback 推给 :9880 客户端。

协议：TCP，每行一个 JSON。下行 task_intent / cancel_task；
      上行 task_accepted|task_rejected|progress|task_terminal。
"""
from __future__ import annotations

import json
import socket
import socketserver
import threading
from typing import Any, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from task_intent import (  # noqa: E402
    SCHEMA_INTENT,
    TaskIntent,
    build_feedback,
    validate_task_intent,
)


class TaskState:
    IDLE = "idle"
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


class _TCPThreadingServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


class ThinHarnessNode(Node):
    def __init__(self) -> None:
        super().__init__("thin_harness")
        self.declare_parameter("downlink_port", 9879)
        self.declare_parameter("uplink_port", 9880)
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("semantic_goal_topic", "/demo/mission/semantic_goal")
        self.declare_parameter("cancel_topic", "/demo/mission/cancel")
        self.declare_parameter("return_home_topic", "/demo/mission/return_home")
        self.declare_parameter("mission_event_topic", "/demo/mission/event")
        self.declare_parameter("mission_start_topic", "/demo/mission/start")
        self.declare_parameter("capability_tags", ["Navigate", "ReturnHome"])
        self.declare_parameter("local_platform", "unitree_go2_edu")
        self.declare_parameter("local_robot_id", "go2_001")
        self.declare_parameter("local_map_id", "demo_live_map")
        self.declare_parameter("local_revision", 1)
        self.declare_parameter("default_deadline_s", 180.0)

        self._cap = set(self.get_parameter("capability_tags").value)
        self._platform = str(self.get_parameter("local_platform").value)
        self._robot_id = str(self.get_parameter("local_robot_id").value)
        self._map_id = str(self.get_parameter("local_map_id").value)
        self._revision = int(self.get_parameter("local_revision").value)
        self._default_deadline = float(self.get_parameter("default_deadline_s").value)

        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("semantic_goal_topic").value), 10
        )
        self._cancel_pub = self.create_publisher(
            String, str(self.get_parameter("cancel_topic").value), 10
        )
        self._rh_pub = self.create_publisher(
            String, str(self.get_parameter("return_home_topic").value), 10
        )
        self._start_pub = self.create_publisher(
            String, str(self.get_parameter("mission_start_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_event_topic").value),
            self._on_mission_event,
            20,
        )

        self._lock = threading.Lock()
        self._intent: Optional[TaskIntent] = None
        self._state = TaskState.IDLE
        self._skill_index = 0
        self._feedback_clients: list[socket.socket] = []
        self._deadline_timer = None

        host = str(self.get_parameter("bind_host").value)
        dport = int(self.get_parameter("downlink_port").value)
        uport = int(self.get_parameter("uplink_port").value)
        node = self

        class IntentHandler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                for line in self.rfile:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        self._reply({"type": "task_rejected", "reason": "REJECT_BAD_JSON"})
                        continue
                    reply = node._handle_downlink(msg)
                    if reply is not None:
                        self._reply(reply)

            def _reply(self, obj: dict[str, Any]) -> None:
                self.wfile.write((json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8"))

        class FeedbackHandler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                with node._lock:
                    node._feedback_clients.append(self.request)
                try:
                    while self.rfile.readline():
                        pass
                finally:
                    with node._lock:
                        if self.request in node._feedback_clients:
                            node._feedback_clients.remove(self.request)

        self._dserver = _TCPThreadingServer((host, dport), IntentHandler)
        self._userver = _TCPThreadingServer((host, uport), FeedbackHandler)
        threading.Thread(target=self._dserver.serve_forever, daemon=True).start()
        threading.Thread(target=self._userver.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"thin_harness ready: task_intent :{dport} → BT semantic chain, "
            f"exec_feedback :{uport}, map={self._map_id} rev={self._revision}"
        )

    # ---- downlink ----
    def _handle_downlink(self, msg: dict[str, Any]) -> Optional[dict[str, Any]]:
        mtype = str(msg.get("type") or "task_intent")
        if mtype == "cancel_task":
            task_id = str(msg.get("task_id") or "")
            self._cancel_task("explicit_cancel")
            return {"type": "task_canceled", "task_id": task_id}
        if mtype != "task_intent" or msg.get("schema") != SCHEMA_INTENT:
            return {"type": "task_rejected", "reason": "REJECT_MESSAGE_TYPE"}

        intent, err = validate_task_intent(
            msg,
            allowed_skills=self._cap,
            local_platform=self._platform,
            local_robot_id=self._robot_id,
        )
        if err or intent is None:
            return {"type": "task_rejected", "task_id": str(msg.get("task_id") or ""), "reason": err}
        if intent.map_id != self._map_id or intent.revision != self._revision:
            return {
                "type": "task_rejected",
                "task_id": intent.task_id,
                "reason": f"REJECT_WORLD_REF:map={intent.map_id}@{intent.revision} "
                f"local={self._map_id}@{self._revision}",
            }
        with self._lock:
            if self._state in (TaskState.ACCEPTED, TaskState.RUNNING):
                return {
                    "type": "task_rejected",
                    "task_id": intent.task_id,
                    "reason": "REJECT_BUSY",
                }
            self._intent = intent
            self._skill_index = 0
            self._state = TaskState.ACCEPTED
        self._push_feedback("task_accepted", detail="intent_validated")
        self._arm_deadline()
        self._dispatch_skill()
        return {"type": "task_accepted", "task_id": intent.task_id}

    def _arm_deadline(self) -> None:
        if self._deadline_timer is not None:
            self._deadline_timer.cancel()
        deadline = self._intent.deadline_s or self._default_deadline  # type: ignore[union-attr]
        self._deadline_timer = threading.Timer(deadline, self._on_deadline)
        self._deadline_timer.daemon = True
        self._deadline_timer.start()

    def _on_deadline(self) -> None:
        self.get_logger().warn("task deadline exceeded → cancel")
        self._cancel_task("deadline")
        self._finish(TaskState.FAILED, "deadline_exceeded")

    def _dispatch_skill(self) -> None:
        intent = self._intent
        if intent is None:
            return
        if self._skill_index >= len(intent.skills):
            self._finish(TaskState.SUCCEEDED, "all_skills_done")
            return
        skill = intent.skills[self._skill_index]
        self._state = TaskState.RUNNING
        if skill.name == "Navigate" and skill.goal is not None:
            goal = PoseStamped()
            goal.header.frame_id = skill.goal.frame_id or intent.frame_id
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = skill.goal.x
            goal.pose.position.y = skill.goal.y
            goal.pose.position.z = skill.goal.z
            goal.pose.orientation.w = 1.0
            self._start_pub.publish(String(data="start"))
            self._goal_pub.publish(goal)
            self._push_feedback(
                "skill_dispatched",
                progress={"skill": skill.name, "index": self._skill_index},
            )
            self.get_logger().info(
                f"skill[{self._skill_index}] Navigate → ({skill.goal.x:.2f},{skill.goal.y:.2f},{skill.goal.z:.2f})"
            )
        elif skill.name == "ReturnHome":
            self._rh_pub.publish(String(data="return_home"))
            self._push_feedback(
                "skill_dispatched",
                progress={"skill": skill.name, "index": self._skill_index},
            )
        else:
            self._finish(TaskState.FAILED, f"skill_not_dispatchable:{skill.name}")

    # ---- uplink / mission events ----
    def _on_mission_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = str(body.get("event", ""))
        with self._lock:
            if self._state != TaskState.RUNNING or self._intent is None:
                return
        progress = {
            "subgoal_index": body.get("subgoal_index"),
            "subgoal_total": body.get("subgoal_total"),
            "skill_index": self._skill_index,
        }
        if event == "mission_succeeded":
            self._skill_index += 1
            self._dispatch_skill()
            return
        if event in ("mission_failed", "mission_rejected"):
            self._finish(TaskState.FAILED, str(body.get("detail", event)))
            return
        if event == "mission_canceled":
            self._finish(TaskState.CANCELED, "mission_canceled")
            return
        if event in (
            "mission_started",
            "subgoal_dispatched",
            "subgoal_reached",
            "pct_path_received",
            "return_home_started",
            "return_home_done",
            "plan_fail_hold",
        ):
            self._push_feedback(event, progress=progress, detail=str(body.get("detail", "")))

    def _cancel_task(self, reason: str) -> None:
        self._cancel_pub.publish(String(data=reason))

    def _finish(self, state: str, detail: str) -> None:
        with self._lock:
            self._state = state
        if self._deadline_timer is not None:
            self._deadline_timer.cancel()
            self._deadline_timer = None
        self._push_feedback("task_terminal", detail=detail, state=state)
        self._intent = None

    def _push_feedback(
        self,
        event: str,
        *,
        progress: Optional[dict[str, Any]] = None,
        detail: str = "",
        state: Optional[str] = None,
    ) -> None:
        intent = self._intent
        task_id = intent.task_id if intent else ""
        msg = build_feedback(
            task_id=task_id,
            state=state or self._state,
            event=event,
            progress=progress,
            detail=detail,
            timestamp_ns=self.get_clock().now().nanoseconds,
        )
        line = (json.dumps(msg, ensure_ascii=False) + "\n").encode("utf-8")
        with self._lock:
            clients = list(self._feedback_clients)
        for conn in clients:
            try:
                conn.sendall(line)
            except OSError:
                with self._lock:
                    if conn in self._feedback_clients:
                        self._feedback_clients.remove(conn)

    def destroy_node(self) -> None:
        self._dserver.shutdown()
        self._userver.shutdown()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ThinHarnessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
