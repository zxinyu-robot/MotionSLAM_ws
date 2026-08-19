#!/usr/bin/env python3
"""零先验玻璃怀疑层：P1 几何/强度 + P2 触觉确认 + 虚拟障碍固化."""
from __future__ import annotations

import json
import math
import time
from typing import Deque, Iterable, List, Optional, Tuple
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String

from glass_suspect_analysis import (
    GlassAnalysisConfig,
    TactileConfig,
    analyze_forward_rays,
    build_glass_wall_patch,
    detect_tactile_bump,
    estimate_obstacle_distance_m,
    fuse_glass_suspect,
)

try:
    from unitree_go.msg import SportModeState
except ImportError:
    SportModeState = None  # type: ignore[misc, assignment]


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GlassSuspectLayer(Node):
    def __init__(self) -> None:
        super().__init__("glass_suspect_layer")
        self.declare_parameter("cloud_topic", "/lio/cloud_world")
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("sport_state_topic", "lf/sportmodestate")
        self.declare_parameter("motor_joint_topic", "/motor_monitor/joint_states")
        self.declare_parameter("obstacle_frame_id", "world")
        self.declare_parameter("virtual_obstacle_topic", "/perception/virtual_obstacles")
        self.declare_parameter("forward_range_m", 1.5)
        self.declare_parameter("forward_half_angle_deg", 35.0)
        self.declare_parameter("near_range_min_m", 0.35)
        self.declare_parameter("sparse_point_threshold", 12)
        self.declare_parameter("range_obstacle_near_m", 0.55)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("ray_bin_deg", 2.0)
        self.declare_parameter("occlusion_gap_min_m", 0.25)
        self.declare_parameter("occlusion_violation_min_bins", 3)
        self.declare_parameter("z_band_min_m", 0.15)
        self.declare_parameter("z_band_max_m", 1.6)
        self.declare_parameter("intensity_enabled", True)
        self.declare_parameter("intensity_high_threshold", 180.0)
        self.declare_parameter("intensity_low_threshold", 20.0)
        self.declare_parameter("intensity_jump_ratio", 4.0)
        self.declare_parameter("specular_isolated_max_points", 2)
        self.declare_parameter("specular_score_threshold", 0.45)
        self.declare_parameter("wall_void_wall_score_threshold", 0.35)
        self.declare_parameter("tactile_enabled", True)
        self.declare_parameter("tau_delta_threshold_nm", 6.0)
        self.declare_parameter("tau_front_min_nm", 12.0)
        self.declare_parameter("imu_impact_threshold", 2.5)
        self.declare_parameter("suspect_window_s", 3.0)
        self.declare_parameter("wall_width_m", 0.6)
        self.declare_parameter("wall_height_m", 1.2)
        self.declare_parameter("wall_z_base_m", 0.15)
        self.declare_parameter("wall_resolution_m", 0.05)
        self.declare_parameter("max_solid_patches", 8)

        self._cfg = GlassAnalysisConfig(
            forward_range_m=float(self.get_parameter("forward_range_m").value),
            forward_half_angle_rad=math.radians(
                float(self.get_parameter("forward_half_angle_deg").value)
            ),
            near_range_min_m=float(self.get_parameter("near_range_min_m").value),
            sparse_point_threshold=int(self.get_parameter("sparse_point_threshold").value),
            range_obstacle_near_m=float(self.get_parameter("range_obstacle_near_m").value),
            ray_bin_deg=float(self.get_parameter("ray_bin_deg").value),
            occlusion_gap_min_m=float(self.get_parameter("occlusion_gap_min_m").value),
            occlusion_violation_min_bins=int(
                self.get_parameter("occlusion_violation_min_bins").value
            ),
            z_band_min_m=float(self.get_parameter("z_band_min_m").value),
            z_band_max_m=float(self.get_parameter("z_band_max_m").value),
            intensity_enabled=bool(self.get_parameter("intensity_enabled").value),
            intensity_high_threshold=float(
                self.get_parameter("intensity_high_threshold").value
            ),
            intensity_low_threshold=float(
                self.get_parameter("intensity_low_threshold").value
            ),
            intensity_jump_ratio=float(self.get_parameter("intensity_jump_ratio").value),
            specular_isolated_max_points=int(
                self.get_parameter("specular_isolated_max_points").value
            ),
            specular_score_threshold=float(
                self.get_parameter("specular_score_threshold").value
            ),
            wall_void_wall_score_threshold=float(
                self.get_parameter("wall_void_wall_score_threshold").value
            ),
        )
        self._tactile_cfg = TactileConfig(
            tau_delta_threshold_nm=float(self.get_parameter("tau_delta_threshold_nm").value),
            tau_front_min_nm=float(self.get_parameter("tau_front_min_nm").value),
            imu_impact_threshold=float(self.get_parameter("imu_impact_threshold").value),
            suspect_window_s=float(self.get_parameter("suspect_window_s").value),
        )
        self._tactile_enabled = bool(self.get_parameter("tactile_enabled").value)
        self._obstacle_frame = str(self.get_parameter("obstacle_frame_id").value)
        self._virtual_topic = str(self.get_parameter("virtual_obstacle_topic").value)
        self._wall_width = float(self.get_parameter("wall_width_m").value)
        self._wall_height = float(self.get_parameter("wall_height_m").value)
        self._wall_z_base = float(self.get_parameter("wall_z_base_m").value)
        self._wall_resolution = float(self.get_parameter("wall_resolution_m").value)
        self._max_patches = int(self.get_parameter("max_solid_patches").value)

        self._odom: Optional[Odometry] = None
        self._cloud: Optional[PointCloud2] = None
        self._min_range_obstacle: Optional[float] = None
        self._cloud_has_intensity: Optional[bool] = None
        self._imu_horiz_acc = 0.0
        self._baseline_imu_horiz = 0.0
        self._max_tau_nm = 0.0
        self._prev_max_tau_nm = 0.0
        self._front_max_tau_nm = 0.0
        self._has_motor_data = False
        self._suspect_times: Deque[float] = deque()
        self._solid_patches: List[List[Tuple[float, float, float]]] = []
        self._glass_confirmed = False
        self._confirm_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        cloud_topic = str(self.get_parameter("cloud_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        if SportModeState is not None:
            sport_topic = str(self.get_parameter("sport_state_topic").value)
            self.create_subscription(SportModeState, sport_topic, self._on_sport, 10)

        if self._tactile_enabled:
            motor_topic = str(self.get_parameter("motor_joint_topic").value)
            self.create_subscription(JointState, motor_topic, self._on_joint_states, 10)

        self._flag_pub = self.create_publisher(Bool, "/perception/glass_suspect/flag", 10)
        self._summary_pub = self.create_publisher(
            String, "/perception/glass_suspect/summary", 10
        )
        self._virtual_pub = self.create_publisher(PointCloud2, self._virtual_topic, 10)

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(max(0.1, 1.0 / hz), self._publish)

        self.get_logger().info(
            "glass_suspect_layer: P1+P2 "
            f"fwd={self._cfg.forward_range_m}m frame={self._obstacle_frame} "
            f"virtual→{self._virtual_topic}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._cloud = msg
        if self._cloud_has_intensity is None:
            self._cloud_has_intensity = any(
                f.name == "intensity" for f in msg.fields
            )
            if not self._cloud_has_intensity:
                self.get_logger().warn(
                    "点云无 intensity 字段，强度层已禁用（仅几何+sport）"
                )
                self._cfg.intensity_enabled = False

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        if msg.header.frame_id:
            self._obstacle_frame = msg.header.frame_id

    def _on_sport(self, msg: SportModeState) -> None:
        vals = [float(v) for v in msg.range_obstacle if v > 1e-3]
        self._min_range_obstacle = min(vals) if vals else None
        ax = float(msg.imu_state.accelerometer[0])
        ay = float(msg.imu_state.accelerometer[1])
        self._imu_horiz_acc = math.hypot(ax, ay)

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.effort:
            return
        efforts = [abs(float(v)) for v in msg.effort]
        self._prev_max_tau_nm = self._max_tau_nm
        self._max_tau_nm = max(efforts) if efforts else 0.0
        front = [
            efforts[i]
            for i in range(min(len(efforts), 6))
        ]
        self._front_max_tau_nm = max(front) if front else 0.0
        self._has_motor_data = True

    def _suspect_recent(self, now: float) -> bool:
        while self._suspect_times and now - self._suspect_times[0] > self._tactile_cfg.suspect_window_s:
            self._suspect_times.popleft()
        return bool(self._suspect_times)

    def _update_tactile_baseline(self, suspect_now: bool) -> None:
        if suspect_now:
            return
        alpha = 0.08
        self._baseline_imu_horiz = (
            (1.0 - alpha) * self._baseline_imu_horiz + alpha * self._imu_horiz_acc
        )

    def _maybe_confirm_tactile(
        self,
        *,
        suspect_now: bool,
        ray_stats: dict,
        pose_x: float,
        pose_y: float,
        pose_z: float,
        yaw: float,
    ) -> None:
        if not self._tactile_enabled or not self._has_motor_data:
            return
        now = time.monotonic()
        if suspect_now:
            self._suspect_times.append(now)

        if not detect_tactile_bump(
            suspect_recent=self._suspect_recent(now),
            max_tau_nm=self._max_tau_nm,
            prev_max_tau_nm=self._prev_max_tau_nm,
            front_max_tau_nm=self._front_max_tau_nm,
            imu_horiz_acc=self._imu_horiz_acc,
            baseline_imu_horiz=self._baseline_imu_horiz,
            cfg=self._tactile_cfg,
        ):
            return

        dist_m = estimate_obstacle_distance_m(
            min_range_obstacle_m=self._min_range_obstacle,
            forward_nearest_m=ray_stats.get("forward_nearest_m"),
        )
        patch = build_glass_wall_patch(
            robot_x=pose_x,
            robot_y=pose_y,
            robot_z=pose_z,
            yaw=yaw,
            distance_m=dist_m,
            width_m=self._wall_width,
            height_m=self._wall_height,
            z_base_m=self._wall_z_base,
            resolution_m=self._wall_resolution,
        )
        self._solid_patches.append(patch)
        if len(self._solid_patches) > self._max_patches:
            self._solid_patches = self._solid_patches[-self._max_patches :]
        self._glass_confirmed = True
        self._confirm_count += 1
        self.get_logger().warn(
            f"玻璃触觉确认 @ {dist_m:.2f}m "
            f"(tau_front={self._front_max_tau_nm:.1f}Nm, patches={len(self._solid_patches)})"
        )

    def _iter_cloud_points(self) -> Iterable[Tuple[float, float, float, Optional[float]]]:
        assert self._cloud is not None
        use_intensity = self._cfg.intensity_enabled and bool(self._cloud_has_intensity)
        if use_intensity:
            for x, y, z, intensity in point_cloud2.read_points(
                self._cloud,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            ):
                yield float(x), float(y), float(z), float(intensity)
        else:
            for x, y, z in point_cloud2.read_points(
                self._cloud, field_names=("x", "y", "z"), skip_nans=True
            ):
                yield float(x), float(y), float(z), None

    def _all_solid_points(self) -> List[Tuple[float, float, float]]:
        out: List[Tuple[float, float, float]] = []
        for patch in self._solid_patches:
            out.extend(patch)
        return out

    def _publish_virtual_obstacles(self) -> None:
        points = self._all_solid_points()
        if not points:
            return
        header = self.get_clock().now().to_msg()
        header.frame_id = self._obstacle_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(header, fields, points)
        self._virtual_pub.publish(cloud)

    def _analyze(self) -> dict:
        near_obstacle = (
            self._min_range_obstacle is not None
            and self._min_range_obstacle < self._cfg.range_obstacle_near_m
        )
        base = {
            "state": "SOLID" if self._solid_patches else "NONE",
            "glass_suspect": bool(self._solid_patches),
            "glass_confirmed": self._glass_confirmed,
            "laser_no_return": False,
            "near_obstacle": near_obstacle,
            "forward_points": 0,
            "forward_near_points": 0,
            "forward_nearest_m": None,
            "min_range_obstacle_m": self._min_range_obstacle,
            "high_cost_ratio": 1.0 if self._solid_patches else 0.0,
            "occlusion_violations": 0,
            "wall_void_wall_score": 0.0,
            "specular_score": 0.0,
            "solid_patch_count": len(self._solid_patches),
            "tactile_confirm_count": self._confirm_count,
            "max_tau_nm": round(self._max_tau_nm, 2),
            "front_max_tau_nm": round(self._front_max_tau_nm, 2),
            "reasons": ["solidified"] if self._solid_patches else [],
        }
        if self._odom is None or self._cloud is None:
            return base

        pose = self._odom.pose.pose
        q = pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        ray_stats = analyze_forward_rays(
            self._iter_cloud_points(),
            robot_x=pose.position.x,
            robot_y=pose.position.y,
            robot_z=pose.position.z,
            yaw=yaw,
            cfg=self._cfg,
        )
        fused = fuse_glass_suspect(
            ray_stats, near_obstacle=near_obstacle, cfg=self._cfg
        )

        suspect_now = bool(fused.get("glass_suspect"))
        self._update_tactile_baseline(suspect_now)
        self._maybe_confirm_tactile(
            suspect_now=suspect_now,
            ray_stats=ray_stats,
            pose_x=pose.position.x,
            pose_y=pose.position.y,
            pose_z=pose.position.z,
            yaw=yaw,
        )

        base.update(ray_stats)
        base.update(fused)

        if self._solid_patches:
            base["state"] = "SOLID"
            base["glass_confirmed"] = True
            base["glass_suspect"] = True
            base["high_cost_ratio"] = 1.0
            if "tactile_confirm" not in base["reasons"]:
                base["reasons"] = list(base["reasons"]) + ["tactile_confirm"]
        elif self._glass_confirmed:
            base["state"] = "CONFIRMED"
            base["glass_confirmed"] = True
            base["glass_suspect"] = True
            base["high_cost_ratio"] = max(float(base.get("high_cost_ratio", 0.0)), 0.95)

        return base

    def _publish(self) -> None:
        stats = self._analyze()
        flag = Bool()
        flag.data = bool(stats["glass_suspect"])
        self._flag_pub.publish(flag)

        summary = String()
        summary.data = json.dumps(stats, ensure_ascii=False)
        self._summary_pub.publish(summary)

        if self._solid_patches:
            self._publish_virtual_obstacles()


def main() -> None:
    rclpy.init()
    node = GlassSuspectLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
