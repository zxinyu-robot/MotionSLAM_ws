#!/usr/bin/env python3
"""Go2 底层电机监控：导航跑着看关节状态 + 异常日志.

订阅 unitree_go/LowState (默认 lf/lowstate)，发布 Foxglove 友好 topic:
  /motor_monitor/joint_states   sensor_msgs/JointState  (q/dq/tau)
  /motor_monitor/temperature    std_msgs/Float32MultiArray
  /motor_monitor/health         std_msgs/Bool
  /motor_monitor/summary        std_msgs/String (JSON 摘要)

异常写入 ROS 日志 + 可选文件 (默认 /tmp/motor_monitor_alerts.log).
"""
from __future__ import annotations

import json
import os
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Deque, Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String

try:
    from unitree_go.msg import LowState
except ImportError:
    LowState = None  # type: ignore[misc, assignment]

NUM_LEG_MOTORS = 12
MOTOR_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]
FOC_MODE = 0x01


@dataclass
class MotorSnapshot:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    temperature: int = 0
    lost: int = 0


@dataclass
class AnomalyEvent:
    kind: str
    motor: str
    detail: str
    value: float
    threshold: float


class MotorMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_monitor_node")

        self.declare_parameter("lowstate_topic", "lf/lowstate")
        self.declare_parameter("joint_states_topic", "/motor_monitor/joint_states")
        self.declare_parameter("temperature_topic", "/motor_monitor/temperature")
        self.declare_parameter("health_topic", "/motor_monitor/health")
        self.declare_parameter("summary_topic", "/motor_monitor/summary")
        self.declare_parameter("summary_hz", 2.0)
        self.declare_parameter("stale_timeout_sec", 1.5)
        self.declare_parameter("temp_warn_c", 70.0)
        self.declare_parameter("temp_crit_c", 80.0)
        self.declare_parameter("tau_warn_nm", 18.0)
        self.declare_parameter("tau_crit_nm", 28.0)
        self.declare_parameter("dq_warn_rad_s", 18.0)
        self.declare_parameter("lost_delta_warn", 1)
        self.declare_parameter("power_v_min", 22.0)
        self.declare_parameter("log_file", "/tmp/motor_monitor_alerts.log")
        self.declare_parameter("enable_log_file", True)
        self.declare_parameter("min_log_interval_sec", 3.0)
        self.declare_parameter("require_foc_mode", False)

        topic = str(self.get_parameter("lowstate_topic").value)
        self._temp_warn = float(self.get_parameter("temp_warn_c").value)
        self._temp_crit = float(self.get_parameter("temp_crit_c").value)
        self._tau_warn = float(self.get_parameter("tau_warn_nm").value)
        self._tau_crit = float(self.get_parameter("tau_crit_nm").value)
        self._dq_warn = float(self.get_parameter("dq_warn_rad_s").value)
        self._lost_delta_warn = int(self.get_parameter("lost_delta_warn").value)
        self._power_v_min = float(self.get_parameter("power_v_min").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self._min_log_interval = float(self.get_parameter("min_log_interval_sec").value)
        self._require_foc = bool(self.get_parameter("require_foc_mode").value)
        self._log_file = str(self.get_parameter("log_file").value)
        self._enable_log_file = bool(self.get_parameter("enable_log_file").value)
        summary_hz = max(0.2, float(self.get_parameter("summary_hz").value))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        js_topic = str(self.get_parameter("joint_states_topic").value)
        temp_topic = str(self.get_parameter("temperature_topic").value)
        health_topic = str(self.get_parameter("health_topic").value)
        summary_topic = str(self.get_parameter("summary_topic").value)

        self._joint_pub = self.create_publisher(JointState, js_topic, pub_qos)
        self._temp_pub = self.create_publisher(Float32MultiArray, temp_topic, pub_qos)
        self._health_pub = self.create_publisher(Bool, health_topic, pub_qos)
        self._summary_pub = self.create_publisher(String, summary_topic, pub_qos)

        self._motors: List[MotorSnapshot] = [MotorSnapshot() for _ in range(NUM_LEG_MOTORS)]
        self._prev_lost: List[int] = [0] * NUM_LEG_MOTORS
        self._power_v = 0.0
        self._power_a = 0.0
        self._last_rx_mono = 0.0
        self._last_summary_mono = 0.0
        self._health_ok = True
        self._active_anomalies: Dict[str, AnomalyEvent] = {}
        self._last_log_mono: Dict[str, float] = {}
        self._recent_events: Deque[str] = deque(maxlen=20)

        if LowState is None:
            self.get_logger().error(
                "unitree_go 未安装，motor_monitor 无法订阅 lowstate"
            )
            self.create_timer(1.0, self._publish_degraded)
            return

        self.create_subscription(LowState, topic, self._on_lowstate, qos)
        self.create_timer(1.0 / summary_hz, self._on_summary_timer)
        self.create_timer(0.5, self._check_stale)
        self.get_logger().info(
            f"motor_monitor: subscribe {topic} -> {js_topic}, "
            f"log_file={self._log_file if self._enable_log_file else 'disabled'}"
        )

    def _publish_degraded(self) -> None:
        msg = Bool()
        msg.data = False
        self._health_pub.publish(msg)

    def _on_lowstate(self, msg: LowState) -> None:
        self._last_rx_mono = time.monotonic()
        self._power_v = float(msg.power_v)
        self._power_a = float(msg.power_a)

        anomalies: List[AnomalyEvent] = []
        positions: List[float] = []
        velocities: List[float] = []
        efforts: List[float] = []
        temperatures: List[float] = []

        for i in range(NUM_LEG_MOTORS):
            m = msg.motor_state[i]
            snap = MotorSnapshot(
                mode=int(m.mode),
                q=float(m.q),
                dq=float(m.dq),
                tau=float(m.tau_est),
                temperature=int(m.temperature),
                lost=int(m.lost),
            )
            self._motors[i] = snap
            positions.append(snap.q)
            velocities.append(snap.dq)
            efforts.append(snap.tau)
            temperatures.append(float(snap.temperature))

            name = MOTOR_NAMES[i]
            anomalies.extend(self._check_motor(i, name, snap))

        if self._power_v > 0.1 and self._power_v < self._power_v_min:
            anomalies.append(
                AnomalyEvent(
                    kind="low_battery_voltage",
                    motor="BMS",
                    detail=f"power_v={self._power_v:.1f}V",
                    value=self._power_v,
                    threshold=self._power_v_min,
                )
            )

        self._publish_joint_state(positions, velocities, efforts)
        self._publish_temperatures(temperatures)
        self._handle_anomalies(anomalies)

    def _check_motor(self, idx: int, name: str, snap: MotorSnapshot) -> List[AnomalyEvent]:
        out: List[AnomalyEvent] = []

        if self._require_foc and snap.mode != FOC_MODE:
            out.append(
                AnomalyEvent(
                    kind="motor_not_foc",
                    motor=name,
                    detail=f"mode=0x{snap.mode:02x}",
                    value=float(snap.mode),
                    threshold=float(FOC_MODE),
                )
            )

        if snap.temperature >= self._temp_crit:
            out.append(
                AnomalyEvent(
                    kind="temp_critical",
                    motor=name,
                    detail=f"temp={snap.temperature}C",
                    value=float(snap.temperature),
                    threshold=self._temp_crit,
                )
            )
        elif snap.temperature >= self._temp_warn:
            out.append(
                AnomalyEvent(
                    kind="temp_high",
                    motor=name,
                    detail=f"temp={snap.temperature}C",
                    value=float(snap.temperature),
                    threshold=self._temp_warn,
                )
            )

        abs_tau = abs(snap.tau)
        if abs_tau >= self._tau_crit:
            out.append(
                AnomalyEvent(
                    kind="tau_critical",
                    motor=name,
                    detail=f"tau={snap.tau:.2f}Nm",
                    value=abs_tau,
                    threshold=self._tau_crit,
                )
            )
        elif abs_tau >= self._tau_warn:
            out.append(
                AnomalyEvent(
                    kind="tau_high",
                    motor=name,
                    detail=f"tau={snap.tau:.2f}Nm",
                    value=abs_tau,
                    threshold=self._tau_warn,
                )
            )

        if abs(snap.dq) >= self._dq_warn:
            out.append(
                AnomalyEvent(
                    kind="dq_high",
                    motor=name,
                    detail=f"dq={snap.dq:.2f}rad/s",
                    value=abs(snap.dq),
                    threshold=self._dq_warn,
                )
            )

        lost_delta = snap.lost - self._prev_lost[idx]
        if lost_delta >= self._lost_delta_warn:
            out.append(
                AnomalyEvent(
                    kind="comm_lost",
                    motor=name,
                    detail=f"lost +{lost_delta} (total={snap.lost})",
                    value=float(lost_delta),
                    threshold=float(self._lost_delta_warn),
                )
            )
        self._prev_lost[idx] = snap.lost
        return out

    def _handle_anomalies(self, anomalies: List[AnomalyEvent]) -> None:
        now = time.monotonic()
        active: Dict[str, AnomalyEvent] = {}
        critical_kinds = {"temp_critical", "tau_critical", "comm_lost", "stale_lowstate"}

        for ev in anomalies:
            key = f"{ev.motor}:{ev.kind}"
            active[key] = ev
            if now - self._last_log_mono.get(key, 0.0) < self._min_log_interval:
                continue
            self._last_log_mono[key] = now
            line = (
                f"{ev.kind} {ev.motor} {ev.detail} "
                f"(value={ev.value:.2g} threshold={ev.threshold:.2g})"
            )
            self._recent_events.appendleft(line)
            if ev.kind in critical_kinds:
                self.get_logger().error(f"[MOTOR] {line}")
            else:
                self.get_logger().warn(f"[MOTOR] {line}")
            self._write_log_file("ERROR" if ev.kind in critical_kinds else "WARN", line)

        cleared = set(self._active_anomalies.keys()) - set(active.keys())
        for key in cleared:
            prev = self._active_anomalies[key]
            line = f"recovered {prev.kind} {prev.motor}"
            if now - self._last_log_mono.get(key + ":clear", 0.0) >= self._min_log_interval:
                self._last_log_mono[key + ":clear"] = now
                self.get_logger().info(f"[MOTOR] {line}")
                self._write_log_file("INFO", line)

        self._active_anomalies = active
        self._health_ok = not any(
            ev.kind in critical_kinds for ev in active.values()
        )
        health = Bool()
        health.data = self._health_ok
        self._health_pub.publish(health)

    def _publish_joint_state(
        self, positions: List[float], velocities: List[float], efforts: List[float]
    ) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "base_link"
        js.name = list(MOTOR_NAMES)
        js.position = positions
        js.velocity = velocities
        js.effort = efforts
        self._joint_pub.publish(js)

    def _publish_temperatures(self, temperatures: List[float]) -> None:
        msg = Float32MultiArray()
        msg.data = temperatures
        self._temp_pub.publish(msg)

    def _check_stale(self) -> None:
        if self._last_rx_mono <= 0.0:
            return
        age = time.monotonic() - self._last_rx_mono
        if age <= self._stale_timeout:
            return
        ev = AnomalyEvent(
            kind="stale_lowstate",
            motor="SYSTEM",
            detail=f"no lowstate for {age:.1f}s",
            value=age,
            threshold=self._stale_timeout,
        )
        self._handle_anomalies([ev])

    def _on_summary_timer(self) -> None:
        if self._last_rx_mono <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_summary_mono < 0.1:
            return
        self._last_summary_mono = now

        max_temp = max((m.temperature for m in self._motors), default=0)
        max_tau = max((abs(m.tau) for m in self._motors), default=0.0)
        max_dq = max((abs(m.dq) for m in self._motors), default=0.0)
        summary = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "health_ok": self._health_ok,
            "power_v": round(self._power_v, 2),
            "power_a": round(self._power_a, 2),
            "max_temp_c": max_temp,
            "max_tau_nm": round(max_tau, 2),
            "max_dq_rad_s": round(max_dq, 2),
            "active_anomalies": len(self._active_anomalies),
            "recent_events": list(self._recent_events)[:5],
        }
        out = String()
        out.data = json.dumps(summary, ensure_ascii=False)
        self._summary_pub.publish(out)

    def _write_log_file(self, level: str, message: str) -> None:
        if not self._enable_log_file or not self._log_file:
            return
        try:
            log_dir = os.path.dirname(self._log_file)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(self._log_file, "a", encoding="utf-8") as f:
                f.write(f"{ts} [{level}] {message}\n")
        except OSError as exc:
            self.get_logger().warn(f"无法写入 motor 日志 {self._log_file}: {exc}")


def main() -> None:
    rclpy.init()
    node = MotorMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
