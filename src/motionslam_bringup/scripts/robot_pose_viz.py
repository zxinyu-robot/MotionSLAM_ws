#!/usr/bin/env python3
"""LIO 位姿 -> Foxglove 友好的狗身可视化.

Super-LIO 的 /lio/odom 为 nav_msgs/Odometry 且 child_frame_id 为空,
Foxglove 3D 对 Odometry 的 Pose 显示不稳定; 本节点转为:

  /lio/pose          geometry_msgs/PoseStamped  (3D 面板 Pose)
  /lio/robot_marker  visualization_msgs/Marker (CUBE, 近似 Go2 机身)
  TF world -> base_link (默认可选仅保留 yaw, 避免地图看起来倾斜)

默认输入 /lio/robo/odom (机身中心); 可改 input_topic:=/lio/odom。

NOTE: Super-LIO 的 world 已重力对齐, 但 /lio/odom 常带 ~10-15° pitch
(雷达/IMU 与机身坐标差异)。Foxglove Fixed Frame 若误选 base_link,
代价图会随机身 pitch 看起来是斜的; 请用 world, 并开启 level_orientation。
"""
import math

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def yaw_only_quaternion(q: Quaternion, yaw_offset_deg: float = 0.0) -> Quaternion:
    """Strip roll/pitch, keep yaw — APP 类水平机身显示."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp) + math.radians(yaw_offset_deg)
    out = Quaternion()
    out.x = 0.0
    out.y = 0.0
    out.z = math.sin(yaw / 2.0)
    out.w = math.cos(yaw / 2.0)
    return out


class RobotPoseViz(Node):
    def __init__(self):
        super().__init__("robot_pose_viz")
        self.declare_parameter("input_topic", "/lio/robo/odom")
        self.declare_parameter("pose_topic", "/lio/pose")
        self.declare_parameter("marker_topic", "/lio/robot_marker")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", "base_link")
        # 仅保留 yaw, 去掉 LIO pitch/roll, 避免 Foxglove 里狗/图相对水平面发斜
        self.declare_parameter("level_orientation", True)
        # 箭头相对 odom yaw 的修正 (度); 若 Foxglove 见狗头与轨迹 X 反向可设 180
        self.declare_parameter("forward_yaw_offset_deg", 0.0)
        # Go2 机身近似尺寸 (m): 长 x 宽 x 高
        self.declare_parameter("body_length", 0.70)
        self.declare_parameter("body_width", 0.30)
        self.declare_parameter("body_height", 0.20)

        input_topic = self.get_parameter("input_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.body_length = float(self.get_parameter("body_length").value)
        self.body_width = float(self.get_parameter("body_width").value)
        self.body_height = float(self.get_parameter("body_height").value)
        self.level_orientation = bool(self.get_parameter("level_orientation").value)
        self.forward_yaw_offset_deg = float(
            self.get_parameter("forward_yaw_offset_deg").value
        )
        # Foxglove 用当前时间戳发布 TF/Marker, 避免与多源 odom 时间交错导致跳动
        self.declare_parameter("use_current_time_for_viz", True)
        self.use_current_time_for_viz = bool(
            self.get_parameter("use_current_time_for_viz").value
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, qos)
        self.marker_pub = self.create_publisher(Marker, marker_topic, qos)
        self.create_subscription(Odometry, input_topic, self._odom_cb, qos)
        level = "yaw-only" if self.level_orientation else "full"
        self.get_logger().info(
            f"RobotPoseViz: {input_topic} -> {pose_topic}, {marker_topic}, "
            f"TF {self.frame_id}->{self.child_frame_id} ({level})"
        )

    def _display_pose(self, msg: Odometry):
        pose = msg.pose.pose
        if not self.level_orientation:
            return pose
        leveled = type(pose)()
        leveled.position = pose.position
        leveled.orientation = yaw_only_quaternion(
            pose.orientation, self.forward_yaw_offset_deg
        )
        return leveled

    def _odom_cb(self, msg: Odometry):
        viz_stamp = self.get_clock().now().to_msg() if self.use_current_time_for_viz else msg.header.stamp
        frame_id = msg.header.frame_id or self.frame_id
        display_pose = self._display_pose(msg)

        pose = PoseStamped()
        pose.header.stamp = viz_stamp
        pose.header.frame_id = frame_id
        pose.pose = display_pose
        self.pose_pub.publish(pose)

        marker = Marker()
        marker.header.stamp = viz_stamp
        marker.header.frame_id = frame_id
        marker.ns = "go2_body"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = display_pose
        marker.scale.x = self.body_length
        marker.scale.y = self.body_width
        marker.scale.z = self.body_height
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.75
        self.marker_pub.publish(marker)

        arrow = Marker()
        arrow.header.stamp = viz_stamp
        arrow.header.frame_id = frame_id
        arrow.ns = "go2_fwd"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = display_pose
        arrow.scale.x = 0.9
        arrow.scale.y = 0.12
        arrow.scale.z = 0.12
        arrow.color.r = 1.0
        arrow.color.g = 0.35
        arrow.color.b = 0.1
        arrow.color.a = 0.95
        self.marker_pub.publish(arrow)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = viz_stamp
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = display_pose.position.x
        tf_msg.transform.translation.y = display_pose.position.y
        tf_msg.transform.translation.z = display_pose.position.z
        tf_msg.transform.rotation = display_pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = RobotPoseViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
