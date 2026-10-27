#!/usr/bin/env python3
"""thin_harness 门禁 4 节点级检查：task_id 幂等/重复、抢占（BT 回执与超时两条路径）、禁区。

前置：另起 thin_harness_node（bind_host:=127.0.0.1 local_revision:=3
areas_file:=<含 restricted_C 的 YAML> preempt_timeout_s:=1.0），本脚本扮演边端与 BT。
"""
import json
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

HOST, DPORT, UPORT = "127.0.0.1", 9879, 9880


def intent(task_id, x=2.0, y=3.0, priority=0, preemptible=False, forbidden=None):
    msg = {
        "schema": "motionslam.task_intent.v1",
        "task_id": task_id,
        "robot": {"platform": "unitree_go2_edu", "sn": "go2_001"},
        "world_ref": {"map_id": "demo_live_map", "floor_id": "f1", "frame_id": "world", "revision": 3},
        "skills": [{"name": "Navigate", "goal": {"x": x, "y": y, "z": 0.35}}],
        "policy": {"deadline_s": 120.0},
        "execution": {"priority": priority, "preemptible": preemptible},
    }
    if forbidden:
        msg["navigation"] = {"forbidden_areas": forbidden}
    return msg


class Bt(Node):
    def __init__(self):
        super().__init__("fake_bt")
        self.goals, self.cancels = [], []
        self.create_subscription(PoseStamped, "/demo/mission/semantic_goal",
                                 lambda m: self.goals.append((m.pose.position.x, m.pose.position.y)), 10)
        self.create_subscription(String, "/demo/mission/cancel", lambda m: self.cancels.append(m.data), 10)
        self.ev = self.create_publisher(String, "/demo/mission/event", 10)

    def emit(self, event):
        self.ev.publish(String(data=json.dumps({"event": event})))


def main():
    rclpy.init()
    bt = Bt()
    threading.Thread(target=rclpy.spin, args=(bt,), daemon=True).start()
    feedback = []
    fb = socket.create_connection((HOST, UPORT))

    def read_fb():
        for line in fb.makefile("r"):
            feedback.append(json.loads(line))
    threading.Thread(target=read_fb, daemon=True).start()
    dl = socket.create_connection((HOST, DPORT))
    rd = dl.makefile("r")
    time.sleep(1.0)

    def send(m):
        dl.sendall((json.dumps(m) + "\n").encode())
        return json.loads(rd.readline())

    def settle(t=0.5):
        time.sleep(t)

    fails = []

    def check(name, ok, info=""):
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {info}")
        if not ok:
            fails.append(name)

    r = send(intent("t-1", preemptible=True)); settle()
    check("accept t-1", r.get("type") == "task_accepted" and len(bt.goals) == 1, r)
    r = send(intent("t-1", preemptible=True)); settle()
    check("idempotent t-1", r.get("idempotent") is True and len(bt.goals) == 1, r)
    r = send(intent("t-2", priority=0))
    check("busy t-2 (priority not higher)", r.get("reason") == "REJECT_BUSY", r)
    r = send(intent("t-f", x=2.0, y=0.5, forbidden=["restricted_C"]))
    check("forbidden goal", str(r.get("reason", "")).startswith("REJECT_GOAL_IN_FORBIDDEN_AREA"), r)
    r = send(intent("t-u", forbidden=["no_such_area"]))
    check("unknown area fail closed", str(r.get("reason", "")).startswith("REJECT_UNKNOWN_AREA"), r)

    r = send(intent("t-3", x=4.0, priority=5, preemptible=True)); settle()
    check("preempt t-1 by t-3", r.get("preempted") == "t-1" and bt.cancels[-1:] == ["preempted"]
          and len(bt.goals) == 1, r)
    r = send(intent("t-4", priority=9))
    check("busy during handoff", r.get("reason") == "REJECT_BUSY", r)
    bt.emit("mission_canceled"); settle()
    check("handoff on mission_canceled", bt.goals[-1:] == [(4.0, 3.0)] and len(bt.goals) == 2, bt.goals)
    r = send(intent("t-1"))
    check("duplicate finished t-1", r.get("reason") == "REJECT_DUPLICATE_TASK_ID", r)

    r = send(intent("t-5", x=6.0, priority=9)); settle(2.0)
    check("preempt timeout handoff", r.get("preempted") == "t-3" and bt.goals[-1:] == [(6.0, 3.0)], bt.goals)
    n_term = sum(1 for f in feedback if f.get("task_id") == "t-5" and f.get("event") == "task_terminal")
    bt.emit("mission_canceled"); settle()
    n_term2 = sum(1 for f in feedback if f.get("task_id") == "t-5" and f.get("event") == "task_terminal")
    check("late cancel ack ignored", n_term == n_term2 == 0, f"terminal t-5: {n_term2}")
    r = send(intent("t-6", priority=99))
    check("t-5 non-preemptible still active", r.get("reason") == "REJECT_BUSY", r)

    print("RESULT", "FAIL " + ",".join(fails) if fails else "ALL PASS")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
