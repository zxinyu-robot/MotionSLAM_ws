#!/usr/bin/env python3
"""vkernel_sim：虚拟 Linux 设备内核 —— 虚拟传感器 + 虚拟执行器。

在云端代替真机硬件：
  - 虚拟雷达：按参数化的房间墙/立柱合成世界系点云 → /lio/cloud_world
  - 虚拟里程计：积分 /motion/command（机体系 vx/vy/yaw_rate）→ /lio/robo/odom
  - 虚拟 Sport 端点：订阅 /api/sport/request 并打印 api_id（仅观测，不闭环）

上层链路（PCT/BT/SCAN/forwarder/Thin Harness）无感运行，不需要真机。
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from motionslam_msgs.msg import MotionCommand

try:
    from unitree_api.msg import Request as SportRequest
except Exception:  # pragma: no cover - unitree 消息缺失时跳过虚拟 Sport 端点
    SportRequest = None


def _walls_from_params(raw) -> list[tuple[float, ...]]:
    # ROS 参数不支持嵌套数组：walls 为扁平 [x1,y1,x2,y2,z0, ...]
    # 兼容旧 4 元组（z0=0）：长度被 5 整除按 5 元组，否则按 4 元组
    flat = [float(v) for v in (raw or [])]
    stride = 5 if len(flat) % 5 == 0 else 4
    walls = []
    for i in range(0, len(flat) - stride + 1, stride):
        item = tuple(flat[i : i + stride])
        walls.append(item if stride == 5 else (*item, 0.0))
    return walls


def _zones_from_params(raw, stride: int) -> list[tuple[float, ...]]:
    flat = [float(v) for v in (raw or [])]
    return [tuple(flat[i : i + stride]) for i in range(0, len(flat) - stride + 1, stride)]


def _height_at(
    x: float,
    y: float,
    ramps: list[tuple[float, ...]],
    floors: list[tuple[float, ...]],
) -> float:
    """地形高度：ramps=[x1,y1,x2,y2,z1,z2,axis]（axis 0=沿x, 1=沿y），
    floors=[x1,y1,x2,y2,z]。先匹配 ramp，再匹配 floor，否则 0。"""
    for x1, y1, x2, y2, z1, z2, axis in ramps:
        if x1 <= x <= x2 and y1 <= y <= y2:
            t = (x - x1) / max(1e-6, x2 - x1) if axis < 0.5 else (y - y1) / max(1e-6, y2 - y1)
            return z1 + (z2 - z1) * min(1.0, max(0.0, t))
    for x1, y1, x2, y2, z in floors:
        if x1 <= x <= x2 and y1 <= y <= y2:
            return z
    return 0.0


def _pillars_from_params(raw) -> list[tuple[float, float, float]]:
    # pillars 为扁平 [cx,cy,r, ...]
    flat = [float(v) for v in (raw or [])]
    pillars = []
    for i in range(0, len(flat) - 2, 3):
        pillars.append(tuple(flat[i : i + 3]))
    return pillars


def build_room_cloud(
    walls: list[tuple[float, ...]],
    pillars: list[tuple[float, float, float]],
    *,
    z_min: float,
    z_max: float,
    step: float,
) -> list[tuple[float, float, float, float]]:
    """把墙段/立柱采样成 XYZI 点列表（world 系）。墙段为 (x1,y1,x2,y2,z0)。"""
    pts: list[tuple[float, float, float, float]] = []
    nz = max(1, int(round((z_max - z_min) / 0.10)))
    z_steps = [z_min + (z_max - z_min) * i / max(1, nz - 1) for i in range(nz)]

    def add_wall(x1: float, y1: float, x2: float, y2: float, z0: float = 0.0) -> None:
        length = math.hypot(x2 - x1, y2 - y1)
        n = max(1, int(length / step))
        for i in range(n + 1):
            t = i / n
            x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
            for z in z_steps:
                pts.append((x, y, z + z0, 40.0))

    for w in walls:
        add_wall(*w)
    for cx, cy, r in pillars:
        circumference = max(1, int(2 * math.pi * r / step))
        for i in range(circumference):
            a = 2 * math.pi * i / circumference
            x, y = cx + r * math.cos(a), cy + r * math.sin(a)
            for z in z_steps:
                pts.append((x, y, z, 60.0))
    return pts


class VKernelSim(Node):
    def __init__(self) -> None:
        super().__init__("vkernel_sim")
        self.declare_parameter("odom_rate_hz", 50.0)
        self.declare_parameter("cloud_rate_hz", 5.0)
        self.declare_parameter("cmd_timeout_s", 0.6)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("cloud_topic", "/lio/cloud_world")
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("motion_command_topic", "/cmd_vel")
        self.declare_parameter("sport_request_topic", "/api/sport/request")
        self.declare_parameter("walls", [0.0])
        self.declare_parameter("pillars", [0.0])
        self.declare_parameter("floors", [0.0])
        self.declare_parameter("ramps", [0.0])
        self.declare_parameter("wall_z_min", 0.30)
        self.declare_parameter("wall_z_max", 1.20)
        self.declare_parameter("sample_step_m", 0.05)
        # HM3D 等真实场景资产（hm3d_to_vkernel.py 产物）：设置后
        # 场景点云替代参数化房间，地面高度改由多层高度图查询
        self.declare_parameter("scene_file", "")
        self.declare_parameter("step_z", 0.45)
        # 帧约定对齐 Super-LIO：odom 原点 = 起点处机体（雷达）高度，场景 z 减去该值
        self.declare_parameter("odom_origin_z", 0.0)
        # 等效 Super-LIO lio.preprocess.blind：雷达近场盲区半径（m）
        self.declare_parameter("blind_m", 0.5)
        # 体素降采样（m），滤掉贴地碎点噪声
        self.declare_parameter("cloud_voxel_m", 0.10)
        # 虚拟雷达可见性：球面角度 z-buffer 取每个方向最近点（遮挡剔除），
        # 否则射线穿透墙/家具把已占据体素 raycast 清空。FOV 为 Mid-360
        # (-7°~52°) 叠加 Go2 安装俯仰 11.8° 后的近似值
        self.declare_parameter("fov_el_min_deg", -19.0)
        self.declare_parameter("fov_el_max_deg", 40.0)
        self.declare_parameter("ang_res_deg", 0.8)
        self.declare_parameter("max_range_m", 12.0)

        self._world = str(self.get_parameter("world_frame").value)
        self._body = str(self.get_parameter("body_frame").value)
        self._x = float(self.get_parameter("start_x").value)
        self._y = float(self.get_parameter("start_y").value)
        self._z = float(self.get_parameter("start_z").value)
        self._yaw = float(self.get_parameter("start_yaw").value)
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._last_cmd_mono = 0.0
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self._last_odom_mono: Optional[float] = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), qos
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("cloud_topic").value), qos
        )
        # 主执行出口：closed_loop_controller → /cmd_vel (Twist)
        self.create_subscription(
            Twist,
            str(self.get_parameter("motion_command_topic").value),
            self._on_motion_command,
            10,
        )
        # 备用出口：MotionCommand（body-frame vx/vy/yaw_rate）
        self.create_subscription(
            MotionCommand,
            "/motion/command",
            self._on_motion_command_body,
            10,
        )
        if SportRequest is not None:
            # forwarder 以 BEST_EFFORT 发布 /api/sport/request（unitree SDK 约定）
            sport_qos = QoSProfile(depth=10)
            sport_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            sport_qos.durability = DurabilityPolicy.VOLATILE
            self.create_subscription(
                SportRequest,
                str(self.get_parameter("sport_request_topic").value),
                self._on_sport_request,
                sport_qos,
            )

        walls = _walls_from_params(list(self.get_parameter("walls").value))
        pillars = _pillars_from_params(list(self.get_parameter("pillars").value))
        self._floors = _zones_from_params(list(self.get_parameter("floors").value), 5)
        self._ramps = _zones_from_params(list(self.get_parameter("ramps").value), 7)
        self._cloud_points = build_room_cloud(
            walls,
            pillars,
            z_min=float(self.get_parameter("wall_z_min").value),
            z_max=float(self.get_parameter("wall_z_max").value),
            step=float(self.get_parameter("sample_step_m").value),
        )
        self._hm: Optional[tuple[np.ndarray, float, np.ndarray]] = None
        self._scene_pts: Optional[np.ndarray] = None
        scene_file = str(self.get_parameter("scene_file").value)
        if scene_file:
            data = np.load(scene_file)
            oz = float(self.get_parameter("odom_origin_z").value)
            pts = data["points"][:, :4].astype(np.float32)
            pts[:, 2] -= oz
            self._scene_pts = pts
            self._cloud_points = []
            self._hm = (
                data["hm_origin"].astype(float),
                float(data["hm_res"]),
                data["hm_levels"].astype(float) - oz,
            )
            self.get_logger().info(
                f"vkernel_sim: scene_file={scene_file} pts={len(self._scene_pts)}"
            )
        if not self._cloud_points:
            self.get_logger().warn("vkernel_sim: 房间为空（walls/pillars 未配置）")

        odom_period = 1.0 / max(1.0, float(self.get_parameter("odom_rate_hz").value))
        cloud_period = 1.0 / max(0.5, float(self.get_parameter("cloud_rate_hz").value))
        self.create_timer(odom_period, self._on_odom_tick)
        self.create_timer(cloud_period, self._on_cloud_tick)
        self.get_logger().info(
            f"vkernel_sim ready: walls={len(walls)} pillars={len(pillars)} "
            f"cloud_pts={len(self._cloud_points)} pose=({self._x:.2f},{self._y:.2f},{self._yaw:.2f})"
        )

    def _on_motion_command(self, msg: Twist) -> None:
        self._vx = float(msg.linear.x)
        self._vy = float(msg.linear.y)
        self._vyaw = float(msg.angular.z)
        self._last_cmd_mono = self.get_clock().now().nanoseconds * 1e-9

    def _on_motion_command_body(self, msg: MotionCommand) -> None:
        self._vx = float(msg.vx)
        self._vy = float(msg.vy)
        self._vyaw = float(msg.yaw_rate)
        self._last_cmd_mono = self.get_clock().now().nanoseconds * 1e-9

    def _on_sport_request(self, msg) -> None:
        ident = getattr(msg, "header", None)
        api = getattr(msg, "parameter", "")
        self.get_logger().info(
            f"vkernel sport <= api request header={ident} param={str(api)[:80]}"
        )

    def _on_odom_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        now = now_ns * 1e-9
        if self._last_odom_mono is None:
            self._last_odom_mono = now
            return
        dt = min(0.1, now - self._last_odom_mono)
        self._last_odom_mono = now
        if self._last_cmd_mono and now - self._last_cmd_mono > self._cmd_timeout:
            self._vx = self._vy = self._vyaw = 0.0
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._x += (self._vx * c - self._vy * s) * dt
        self._y += (self._vx * s + self._vy * c) * dt
        self._yaw = math.atan2(math.sin(self._yaw + self._vyaw * dt), math.cos(self._yaw + self._vyaw * dt))
        self._z = self._ground_z()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world
        msg.child_frame_id = self._body
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = self._z
        msg.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        msg.twist.twist.linear.x = self._vx
        msg.twist.twist.linear.y = self._vy
        msg.twist.twist.angular.z = self._vyaw
        self._odom_pub.publish(msg)

    def _ground_z(self) -> float:
        """地面 z：优先场景高度图（取距当前 z 最近的候选层，模拟沿楼梯逐层过渡），
        否则回退参数化 ramps/floors。"""
        if self._hm is not None:
            (ox, oy), res, levels = self._hm
            cx = int((self._x - ox) / res)
            cy = int((self._y - oy) / res)
            if 0 <= cy < levels.shape[0] and 0 <= cx < levels.shape[1]:
                cands = levels[cy, cx][np.isfinite(levels[cy, cx])]
                if cands.size:
                    step = float(self.get_parameter("step_z").value)
                    # 双向步高约束：只允许 |Δz|<=step 的层（台阶 ~0.2m 逐级可上可下；
                    # 中庭/梯井空洞处底层不可瞬移跌落——等效地面连续约束）
                    reach = cands[np.abs(cands - self._z) <= step]
                    if reach.size:
                        return float(reach[np.argmin(np.abs(reach - self._z))])
                    return self._z  # 无可达层：视为撞墙/悬空，保持当前高度
            return self._z
        return _height_at(self._x, self._y, self._ramps, self._floors)

    _FIELDS = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    def _visible_points(self) -> np.ndarray:
        """以当前雷达位姿做遮挡剔除 + 盲区/量程/FOV 裁剪。"""
        pts = self._scene_pts
        d = pts[:, :3] - np.array([self._x, self._y, self._z], np.float32)
        r = np.linalg.norm(d, axis=1)
        blind = float(self.get_parameter("blind_m").value)
        keep = (r > max(blind, 1e-3)) & (r < float(self.get_parameter("max_range_m").value))
        pts, d, r = pts[keep], d[keep], r[keep]
        el = np.degrees(np.arcsin(np.clip(d[:, 2] / r, -1.0, 1.0)))
        el_min = float(self.get_parameter("fov_el_min_deg").value)
        fov = (el >= el_min) & (el <= float(self.get_parameter("fov_el_max_deg").value))
        pts, d, r, el = pts[fov], d[fov], r[fov], el[fov]
        res = float(self.get_parameter("ang_res_deg").value)
        az = np.degrees(np.arctan2(d[:, 1], d[:, 0])) + 180.0
        n_el = int(180.0 / res) + 1
        key = (az / res).astype(np.int64) * n_el + ((el - el_min) / res).astype(np.int64)
        order = np.lexsort((r, key))
        _, first = np.unique(key[order], return_index=True)
        return pts[order[first]]

    def _filtered_scene_msg(self) -> PointCloud2:
        """等效真实 lidar 前端：可见性(遮挡+blind+FOV) → 本层地面分割 +
        通行高度带（虚拟天花板）→ 体素降采样滤碎点。"""
        level = self._ground_z()
        surface_z = level - 0.35  # hm_levels 存 body 高度 = 地表+0.35
        band = self._visible_points()
        rel = band[:, 2] - surface_z
        band = band[(rel > 0.10) & (rel < 1.60)]
        vox = float(self.get_parameter("cloud_voxel_m").value)
        if vox > 0.0 and len(band):
            key = np.floor(band[:, :3] / vox).astype(np.int64)
            _, idx = np.unique(key, axis=0, return_index=True)
            band = band[np.sort(idx)]
        msg = PointCloud2()
        msg.header.frame_id = self._world
        msg.height = 1
        msg.width = int(len(band))
        msg.fields = self._FIELDS
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(band, dtype=np.float32).tobytes()
        return msg

    def _on_cloud_tick(self) -> None:
        if self._scene_pts is not None:
            msg = self._filtered_scene_msg()
            msg.header.stamp = self.get_clock().now().to_msg()
            self._cloud_pub.publish(msg)
            return
        if not self._cloud_points:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._world
        msg = pc2.create_cloud(header, self._FIELDS, self._cloud_points)
        self._cloud_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = VKernelSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
