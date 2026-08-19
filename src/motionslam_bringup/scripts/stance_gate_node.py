#!/usr/bin/env python3
"""足式着地门控：稳定着地期才允许点云进入 local_map / scan_planner."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from unitree_go.msg import SportModeState
except ImportError:
    SportModeState = None  # type: ignore[misc, assignment]


class StanceGateNode(Node):
    def __init__(self) -> None:
        super().__init__("stance_gate_node")
        self.declare_parameter("sport_state_topic", "lf/sportmodestate")
        self.declare_parameter("min_foot_force", 80)
        self.declare_parameter("min_feet_on_ground", 3)
        self.declare_parameter("stable_frames", 3)

        topic = str(self.get_parameter("sport_state_topic").value)
        self._min_force = int(self.get_parameter("min_foot_force").value)
        self._min_feet = int(self.get_parameter("min_feet_on_ground").value)
        self._stable_frames = int(self.get_parameter("stable_frames").value)
        self._stable_count = 0
        self._stance_ok = True
        self._force_unavailable_logged = False
        self._sport_msg_count = 0

        self._pub = self.create_publisher(Bool, "/local_map/stance_ok", 10)

        if SportModeState is None:
            self.get_logger().warn(
                "unitree_go 不可用，stance_gate 恒为 true（无着地滤波）"
            )
            self.create_timer(0.5, self._publish_always_ok)
            return

        self.create_subscription(SportModeState, topic, self._on_sport, 10)
        self.get_logger().info(
            f"stance_gate topic={topic} min_force={self._min_force} "
            f"min_feet={self._min_feet} stable={self._stable_frames}"
        )

    def _publish_always_ok(self) -> None:
        msg = Bool()
        msg.data = True
        self._pub.publish(msg)

    def _on_sport(self, msg: SportModeState) -> None:
        self._sport_msg_count += 1
        forces = list(msg.foot_force)
        max_force = max(forces) if forces else 0
        # 部分 GO2 固件 lf/sportmodestate.foot_force 恒为 0，无法做着地判定
        if self._sport_msg_count >= self._stable_frames and max_force == 0:
            if not self._force_unavailable_logged:
                self.get_logger().warn(
                    "foot_force 全零，视为传感器不可用 → stance_ok 恒 true"
                )
                self._force_unavailable_logged = True
            self._stance_ok = True
        else:
            feet_on = sum(1 for f in forces if f >= self._min_force)
            if feet_on >= self._min_feet:
                self._stable_count = min(
                    self._stable_count + 1, self._stable_frames + 1
                )
            else:
                self._stable_count = 0
            self._stance_ok = self._stable_count >= self._stable_frames
        out = Bool()
        out.data = self._stance_ok
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = StanceGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
