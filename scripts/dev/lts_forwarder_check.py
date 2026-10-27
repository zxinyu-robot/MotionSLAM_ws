#!/usr/bin/env python3
"""cmd_vel_forwarder 地形安全模式检查（容器内，栈需 with_forwarder:=true 且无 vkernel 发 /terrain/decision）。

向 /motion/command 发 vx=0.5，依次注入 Normal/ReduceSpeed/HoldPosition/停发，
打印 /api/sport/request Move(1008) 的 vx。期望 0.5/0.5/0.2/0/0.5/0。
"""
import json, time, rclpy
from rclpy.node import Node
from motionslam_msgs.msg import MotionCommand, TerrainDecision
from unitree_api.msg import Request
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = Node("lts_fwd_test")
cmd = n.create_publisher(MotionCommand, "/motion/command", 10)
dec = n.create_publisher(TerrainDecision, "/terrain/decision", 10)
last = {}
def on_req(m):
    if m.header.identity.api_id == 1008:
        last["vx"] = json.loads(m.parameter).get("x")
n.create_subscription(Request, "/api/sport/request", on_req, qos_profile_sensor_data)
def phase(name, mode, secs):
    t0 = time.time()
    while time.time() - t0 < secs:
        c = MotionCommand(); c.vx = 0.5; cmd.publish(c)
        if mode is not None:
            d = TerrainDecision(); d.header.stamp = n.get_clock().now().to_msg(); d.safety_mode = mode; dec.publish(d)
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f"{name}: last Move vx={last.get('vx')}", flush=True)
phase("warmup/no-decision", None, 3.0)
phase("NORMAL", 0, 1.5)
phase("REDUCE_SPEED", 1, 1.5)
phase("HOLD_POSITION", 2, 1.5)
phase("NORMAL again", 0, 1.5)
phase("decision stale(>1.5s)", None, 2.5)

n.destroy_node()
rclpy.shutdown()
