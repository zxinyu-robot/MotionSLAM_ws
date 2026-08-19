#!/usr/bin/env python3
"""边侧 MVPI1 mock planner v0（P2-6～8 子集）。

监听 GO2 上行 9877 subgraph / 9880 feedback；按 query 下发 SEARCH；
收到探索 uplink 或 search_frontier_reached 后 mock re-detect → 下发 NAV ActionGroup。

仅输出 SemanticDirective / ActionGroup → GO2 :9879，不下发 motion。
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))
try:
    from mvpi1_payloads import (  # noqa: E402
        build_action_group,
        build_semantic_directive,
        new_command_id,
        send_json_line,
    )
except ImportError:
    ROOT = str(_SCRIPT_DIR.parents[2])
    sys.path.insert(0, ROOT)
    from scripts.offline.mvpi1_payloads import (  # noqa: E402
        build_action_group,
        build_semantic_directive,
        new_command_id,
        send_json_line,
    )


@dataclass
class SearchDispatchAck:
    command_id: str = ""
    query: str = ""
    sent_at: float = 0.0
    task_accepted: bool = False
    frontier_selected: bool = False
    action_group_retries: int = 0
    confirmed: bool = False
    last_retry_at: float = 0.0
    last_reject_reason: str = ""


@dataclass
class PlannerState:
    phase: str = "idle"
    query: str = ""
    search_command_id: str = ""
    nav_command_id: str = ""
    uplink_count: int = 0
    frontier_reached_count: int = 0
    last_subgraph: dict[str, Any] = field(default_factory=dict)
    last_robot_xy: tuple[float, float] = (0.0, 0.0)
    last_robot_yaw: float = 0.0
    last_frontier_ids: list[str] = field(default_factory=list)
    search_ack: SearchDispatchAck = field(default_factory=SearchDispatchAck)
    last_search_directive: dict[str, Any] = field(default_factory=dict)
    last_search_action_group: dict[str, Any] = field(default_factory=dict)


class EdgeMvpi1Planner:
    def __init__(
        self,
        *,
        go2_host: str,
        go2_port: int,
        session_id: str,
        redeploy_after_uplinks: int,
        redeploy_after_reached: int,
        nav_offset_m: float,
        ack_timeout_s: float,
        max_action_group_retries: int,
        lock: threading.Lock,
    ) -> None:
        self._go2_host = go2_host
        self._go2_port = go2_port
        self._session_id = session_id
        self._redeploy_uplinks = redeploy_after_uplinks
        self._redeploy_reached = redeploy_after_reached
        self._nav_offset_m = nav_offset_m
        self._ack_timeout_s = ack_timeout_s
        self._max_action_group_retries = max_action_group_retries
        self._lock = lock
        self.state = PlannerState()

    def _frontier_ids_for_search_locked(self) -> list[str]:
        frontier_ids = list(self.state.last_frontier_ids)
        if not frontier_ids and self.state.last_subgraph:
            raw = self.state.last_subgraph.get("frontiers") or []
            frontier_ids = [
                str(item.get("frontier_id"))
                for item in raw
                if isinstance(item, dict) and item.get("frontier_id")
            ][:3]
        return frontier_ids

    def _send_search_payloads_locked(self, *, reason: str) -> None:
        frontier_ids = self._frontier_ids_for_search_locked()
        directive = build_semantic_directive(
            query=self.state.query,
            session_id=self._session_id,
            map_version=0,
            context_generation=0,
            search_mode="explore",
            candidate_frontier_ids=frontier_ids,
            confidence=0.55,
            command_id=self.state.search_command_id,
        )
        action_group = build_action_group(
            query=self.state.query,
            session_id=self._session_id,
            map_version=0,
            context_generation=0,
            include_search=True,
            include_nav=False,
            command_id=self.state.search_command_id,
        )
        self.state.last_search_directive = directive
        self.state.last_search_action_group = action_group
        send_json_line(self._go2_host, self._go2_port, directive)
        send_json_line(self._go2_host, self._go2_port, action_group)
        now = time.time()
        self.state.search_ack.sent_at = now
        self.state.search_ack.last_retry_at = now
        print(
            f"[planner] SEARCH sent ({reason}) command_id={self.state.search_command_id} "
            f"frontiers={frontier_ids} "
            f"(await task_accepted + frontier_selected)",
            flush=True,
        )

    def _resend_action_group_locked(self, *, reason: str) -> None:
        if not self.state.last_search_action_group:
            self._send_search_payloads_locked(reason=reason)
            return
        self.state.search_ack.action_group_retries += 1
        self.state.search_ack.last_retry_at = time.time()
        send_json_line(self._go2_host, self._go2_port, self.state.last_search_action_group)
        print(
            f"[planner] SEARCH retry ActionGroup "
            f"#{self.state.search_ack.action_group_retries} "
            f"command_id={self.state.search_command_id} reason={reason}",
            flush=True,
        )

    def _mark_search_confirmed_locked(self) -> None:
        if self.state.search_ack.confirmed:
            return
        self.state.search_ack.confirmed = True
        print(
            f"[planner] SEARCH confirmed command_id={self.state.search_command_id} "
            f"(task_accepted + frontier_selected)",
            flush=True,
        )

    def _feedback_command_id(self, obj: dict[str, Any]) -> str:
        for key in ("command_id", "search_command_id"):
            value = obj.get(key)
            if value:
                return str(value)
        return ""

    def _matches_active_search(self, obj: dict[str, Any]) -> bool:
        if self.state.phase != "search" or not self.state.search_command_id:
            return False
        cid = self._feedback_command_id(obj)
        if cid and cid != self.state.search_command_id:
            return False
        return True

    def start_search(self, query: str) -> None:
        with self._lock:
            preserved = PlannerState(
                last_subgraph=self.state.last_subgraph,
                last_robot_xy=self.state.last_robot_xy,
                last_robot_yaw=self.state.last_robot_yaw,
                last_frontier_ids=list(self.state.last_frontier_ids),
            )
            self.state = preserved
            self.state.phase = "search"
            self.state.query = query
            self.state.search_command_id = new_command_id("search")
            self.state.search_ack = SearchDispatchAck(
                command_id=self.state.search_command_id,
                query=query,
            )
            self._send_search_payloads_locked(reason="start_search")

    def poll_search_ack(self) -> None:
        with self._lock:
            if self.state.phase != "search" or self.state.search_ack.confirmed:
                return
            ack = self.state.search_ack
            now = time.time()
            if ack.task_accepted and ack.frontier_selected:
                self._mark_search_confirmed_locked()
                return
            if ack.action_group_retries >= self._max_action_group_retries:
                print(
                    f"[planner] SEARCH unconfirmed after "
                    f"{ack.action_group_retries} ActionGroup retries "
                    f"command_id={self.state.search_command_id}",
                    flush=True,
                )
                return
            retry_reason = ""
            if ack.last_reject_reason:
                retry_reason = ack.last_reject_reason
            elif now - ack.sent_at >= self._ack_timeout_s:
                retry_reason = "ack_timeout"
            if not retry_reason:
                return
            if now - ack.last_retry_at < self._ack_timeout_s:
                return
            self._resend_action_group_locked(reason=retry_reason)
            ack.last_reject_reason = ""

    def _robot_from_subgraph(self, obj: dict[str, Any]) -> tuple[float, float, float]:
        robot = obj.get("robot") or obj.get("robot_pose") or {}
        if isinstance(robot, dict):
            x = float(robot.get("x", 0.0))
            y = float(robot.get("y", 0.0))
            yaw_deg = float(robot.get("yaw_deg", 0.0))
            return x, y, math.radians(yaw_deg)
        nodes = obj.get("nodes") or []
        for node in nodes:
            if isinstance(node, dict) and node.get("node_id") == "robot":
                return float(node.get("x", 0.0)), float(node.get("y", 0.0)), 0.0
        return self.state.last_robot_xy[0], self.state.last_robot_xy[1], self.state.last_robot_yaw

    def on_subgraph(self, obj: dict[str, Any]) -> None:
        with self._lock:
            self.state.last_subgraph = obj
            rx, ry, yaw = self._robot_from_subgraph(obj)
            self.state.last_robot_xy = (rx, ry)
            self.state.last_robot_yaw = yaw
            raw = obj.get("frontiers") or []
            self.state.last_frontier_ids = [
                str(item.get("frontier_id"))
                for item in raw
                if isinstance(item, dict) and item.get("frontier_id")
            ]
            if self.state.phase != "search":
                return
            self.state.uplink_count += 1
            print(
                f"[planner] subgraph v={obj.get('version')} "
                f"frontiers={len(raw)} uplink#{self.state.uplink_count}",
                flush=True,
            )
            if self.state.uplink_count >= self._redeploy_uplinks:
                self._dispatch_nav_locked(reason="uplink_threshold")

    def on_feedback(self, obj: dict[str, Any]) -> None:
        event = str(obj.get("event", ""))
        with self._lock:
            if self.state.phase == "search" and self._matches_active_search(obj):
                ack = self.state.search_ack
                if event == "task_accepted":
                    ack.task_accepted = True
                    print(
                        f"[planner] feedback task_accepted command_id="
                        f"{self.state.search_command_id}",
                        flush=True,
                    )
                elif event == "task_rejected":
                    reason = str(obj.get("reason", "task_rejected"))
                    ack.last_reject_reason = reason
                    print(
                        f"[planner] feedback task_rejected reason={reason} "
                        f"command_id={self.state.search_command_id}",
                        flush=True,
                    )
                elif event == "frontier_selected":
                    ack.frontier_selected = True
                    print(
                        f"[planner] feedback frontier_selected "
                        f"frontier_id={obj.get('frontier_id', '')} "
                        f"command_id={self.state.search_command_id}",
                        flush=True,
                    )
                elif event in ("lifecycle_inactive", "mission_failed"):
                    ack.last_reject_reason = event
                    print(
                        f"[planner] feedback {event} → will retry ActionGroup "
                        f"command_id={self.state.search_command_id}",
                        flush=True,
                    )
                if ack.task_accepted and ack.frontier_selected:
                    self._mark_search_confirmed_locked()

            if self.state.phase != "search":
                return
            if event == "search_frontier_reached":
                self.state.frontier_reached_count += 1
                print(
                    f"[planner] feedback {event} count={self.state.frontier_reached_count}",
                    flush=True,
                )
                if self.state.frontier_reached_count >= self._redeploy_reached:
                    self._dispatch_nav_locked(reason="frontier_reached")

    def _dispatch_nav_locked(self, reason: str) -> None:
        if self.state.phase == "nav_sent":
            return
        rx, ry = self.state.last_robot_xy
        yaw = self.state.last_robot_yaw
        gx = rx + self._nav_offset_m * math.cos(yaw)
        gy = ry + self._nav_offset_m * math.sin(yaw)
        self.state.nav_command_id = new_command_id("nav")
        self.state.phase = "nav_sent"

        nav_directive = build_semantic_directive(
            query=self.state.query,
            session_id=self._session_id,
            map_version=0,
            context_generation=0,
            search_mode="navigate",
            confidence=0.91,
            command_id=self.state.nav_command_id,
        )
        nav_group = build_action_group(
            query=self.state.query,
            session_id=self._session_id,
            map_version=0,
            context_generation=0,
            include_search=False,
            include_nav=True,
            nav_x=gx,
            nav_y=gy,
            command_id=self.state.nav_command_id,
        )
        send_json_line(self._go2_host, self._go2_port, nav_directive)
        send_json_line(self._go2_host, self._go2_port, nav_group)
        print(
            f"[planner] NAV dispatched ({reason}) command_id={self.state.nav_command_id} "
            f"goal=({gx:.2f},{gy:.2f})",
            flush=True,
        )


def _serve_json_lines(
    name: str,
    host: str,
    port: int,
    on_line: Callable[[dict[str, Any]], None],
) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"[{name}] listening {host}:{port}", flush=True)
    buf = b""
    while True:
        conn, addr = srv.accept()
        print(f"[{name}] client {addr}", flush=True)
        buf = b""
        try:
            while True:
                chunk = conn.recv(65536)
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
                        continue
                    on_line(obj)
        finally:
            conn.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Edge MVPI1 mock planner v0")
    parser.add_argument("--host", default="0.0.0.0", help="listen host for uplink")
    parser.add_argument("--subgraph-port", type=int, default=9877)
    parser.add_argument("--feedback-port", type=int, default=9880)
    parser.add_argument("--go2-host", default="127.0.0.1", help="GO2 directive_receiver")
    parser.add_argument("--go2-port", type=int, default=9879)
    parser.add_argument("--session-id", default="demo_live")
    parser.add_argument("--query", default="找红色灭火器")
    parser.add_argument("--redeploy-uplinks", type=int, default=2)
    parser.add_argument("--redeploy-reached", type=int, default=1)
    parser.add_argument("--nav-offset-m", type=float, default=2.0)
    parser.add_argument(
        "--ack-timeout-s",
        type=float,
        default=8.0,
        help="未收到 task_accepted+frontier_selected 时重发 ActionGroup 间隔",
    )
    parser.add_argument(
        "--max-action-group-retries",
        type=int,
        default=6,
        help="SEARCH 未确认时 ActionGroup 最大重发次数",
    )
    parser.add_argument(
        "--start-search",
        action="store_true",
        help="启动后立即向 GO2 下发 SEARCH（需 GO2 栈已 active）",
    )
    args = parser.parse_args()

    lock = threading.Lock()
    planner = EdgeMvpi1Planner(
        go2_host=args.go2_host,
        go2_port=args.go2_port,
        session_id=args.session_id,
        redeploy_after_uplinks=args.redeploy_uplinks,
        redeploy_after_reached=args.redeploy_reached,
        nav_offset_m=args.nav_offset_m,
        ack_timeout_s=args.ack_timeout_s,
        max_action_group_retries=args.max_action_group_retries,
        lock=lock,
    )

    t_sub = threading.Thread(
        target=_serve_json_lines,
        args=("subgraph", args.host, args.subgraph_port, planner.on_subgraph),
        daemon=True,
    )
    t_fb = threading.Thread(
        target=_serve_json_lines,
        args=("feedback", args.host, args.feedback_port, planner.on_feedback),
        daemon=True,
    )
    t_sub.start()
    t_fb.start()

    if args.start_search:
        time.sleep(0.5)
        planner.start_search(args.query)

    print(
        "[planner] ready. SEARCH ack = task_accepted + frontier_selected; "
        "failures retry ActionGroup only",
        flush=True,
    )
    try:
        while True:
            planner.poll_search_ack()
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[planner] stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
