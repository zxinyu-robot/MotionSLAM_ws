"""零先验玻璃怀疑：纯几何/强度分析（无 ROS 依赖，可单元测试）."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple


@dataclass
class RayBin:
    nearest_m: float = math.inf
    farthest_m: float = 0.0
    nearest_z: float = 0.0
    point_count: int = 0
    intensities: List[float] = field(default_factory=list)


@dataclass
class GlassAnalysisConfig:
    forward_range_m: float = 1.5
    forward_half_angle_rad: float = math.radians(35.0)
    near_range_min_m: float = 0.35
    min_point_dist_m: float = 0.05
    sparse_point_threshold: int = 12
    range_obstacle_near_m: float = 0.55
    ray_bin_deg: float = 2.0
    occlusion_gap_min_m: float = 0.25
    occlusion_violation_min_bins: int = 3
    z_band_min_m: float = 0.15
    z_band_max_m: float = 1.6
    intensity_enabled: bool = True
    intensity_high_threshold: float = 180.0
    intensity_low_threshold: float = 20.0
    intensity_jump_ratio: float = 4.0
    specular_isolated_max_points: int = 2
    specular_score_threshold: float = 0.45
    wall_void_wall_score_threshold: float = 0.35


@dataclass
class TactileConfig:
    tau_delta_threshold_nm: float = 6.0
    tau_front_min_nm: float = 12.0
    imu_impact_threshold: float = 2.5
    suspect_window_s: float = 3.0
    front_motor_indices: Tuple[int, ...] = (0, 1, 2, 3, 4, 5)


# FR_hip..FL_calf in motor_monitor MOTOR_NAMES order
FRONT_LEG_MOTOR_INDICES = (0, 1, 2, 3, 4, 5)


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _bin_index(bearing_rad: float, half_angle_rad: float, ray_bin_rad: float) -> Optional[int]:
    if abs(bearing_rad) > half_angle_rad:
        return None
    return int((bearing_rad + half_angle_rad) / ray_bin_rad)


def analyze_forward_rays(
    points: Iterable[Tuple[float, float, float, Optional[float]]],
    *,
    robot_x: float,
    robot_y: float,
    robot_z: float,
    yaw: float,
    cfg: GlassAnalysisConfig,
) -> dict:
    """按方位角分 bin，检测 Wall-Void-Wall 遮挡矛盾与强度镜面特征."""
    ray_bin_rad = math.radians(max(cfg.ray_bin_deg, 0.5))
    num_bins = max(1, int(math.ceil((2.0 * cfg.forward_half_angle_rad) / ray_bin_rad)))
    bins: List[RayBin] = [RayBin() for _ in range(num_bins)]

    fwd_total = 0
    fwd_near = 0
    has_intensity = False

    for x, y, z, intensity in points:
        dx = x - robot_x
        dy = y - robot_y
        dist = math.hypot(dx, dy)
        if dist > cfg.forward_range_m or dist < cfg.min_point_dist_m:
            continue

        bearing = _normalize_angle(math.atan2(dy, dx) - yaw)
        idx = _bin_index(bearing, cfg.forward_half_angle_rad, ray_bin_rad)
        if idx is None:
            continue

        rel_z = z - robot_z
        if rel_z < cfg.z_band_min_m or rel_z > cfg.z_band_max_m:
            continue

        fwd_total += 1
        if dist >= cfg.near_range_min_m:
            fwd_near += 1

        rb = bins[idx]
        rb.point_count += 1
        if intensity is not None:
            has_intensity = True
            rb.intensities.append(float(intensity))

        if dist < rb.nearest_m:
            rb.nearest_m = dist
            rb.nearest_z = rel_z
        if dist > rb.farthest_m:
            rb.farthest_m = dist

    occlusion_violations = 0
    violation_bins: List[int] = []
    for i, rb in enumerate(bins):
        if rb.point_count < 2 or not math.isfinite(rb.nearest_m):
            continue
        gap = rb.farthest_m - rb.nearest_m
        if gap >= cfg.occlusion_gap_min_m and rb.nearest_m >= cfg.near_range_min_m:
            occlusion_violations += 1
            violation_bins.append(i)

    wall_void_wall_score = min(
        1.0, occlusion_violations / max(1, cfg.occlusion_violation_min_bins)
    )

    specular_bins = 0
    intensity_jump_bins = 0
    if cfg.intensity_enabled and has_intensity:
        for rb in bins:
            if not rb.intensities:
                continue
            lo = min(rb.intensities)
            hi = max(rb.intensities)
            if lo > 1e-3 and hi / lo >= cfg.intensity_jump_ratio:
                intensity_jump_bins += 1
            if (
                rb.point_count <= cfg.specular_isolated_max_points
                and hi >= cfg.intensity_high_threshold
                and rb.nearest_m >= cfg.near_range_min_m
            ):
                specular_bins += 1

    specular_score = 0.0
    if has_intensity and num_bins > 0:
        specular_score = min(
            1.0,
            (specular_bins + 0.5 * intensity_jump_bins)
            / max(1.0, num_bins * 0.08),
        )

    laser_no_return = fwd_near < cfg.sparse_point_threshold
    geometry_suspect = (
        occlusion_violations >= cfg.occlusion_violation_min_bins
        or wall_void_wall_score >= cfg.wall_void_wall_score_threshold
    )
    intensity_suspect = specular_score >= cfg.specular_score_threshold

    forward_nearest_m = math.inf
    for rb in bins:
        if math.isfinite(rb.nearest_m):
            forward_nearest_m = min(forward_nearest_m, rb.nearest_m)

    return {
        "forward_points": fwd_total,
        "forward_near_points": fwd_near,
        "forward_nearest_m": (
            None if not math.isfinite(forward_nearest_m) else round(forward_nearest_m, 3)
        ),
        "laser_no_return": laser_no_return,
        "occlusion_violations": occlusion_violations,
        "violation_bin_count": len(violation_bins),
        "wall_void_wall_score": round(wall_void_wall_score, 3),
        "specular_bins": specular_bins,
        "intensity_jump_bins": intensity_jump_bins,
        "specular_score": round(specular_score, 3),
        "has_intensity": has_intensity,
        "geometry_suspect": geometry_suspect,
        "intensity_suspect": intensity_suspect,
        "ray_bin_count": num_bins,
    }


def fuse_glass_suspect(
    ray_stats: dict,
    *,
    near_obstacle: bool,
    cfg: GlassAnalysisConfig,
) -> dict:
    """融合几何/强度/稀疏+sport 信号，输出 BT 可用的 suspect 状态."""
    geometry = bool(ray_stats.get("geometry_suspect"))
    intensity = bool(ray_stats.get("intensity_suspect"))
    laser_no_return = bool(ray_stats.get("laser_no_return"))
    fwd_total = int(ray_stats.get("forward_points", 0))

    glass_suspect = False
    high_cost_ratio = 0.0
    reasons: List[str] = []

    if geometry and (laser_no_return or near_obstacle):
        glass_suspect = True
        high_cost_ratio = 1.0
        reasons.append("occlusion_conflict")
    elif geometry:
        glass_suspect = True
        high_cost_ratio = 0.85
        reasons.append("wall_void_wall")
    elif intensity and (laser_no_return or near_obstacle):
        glass_suspect = True
        high_cost_ratio = 0.75
        reasons.append("specular_sparse")
    elif laser_no_return and near_obstacle:
        glass_suspect = True
        high_cost_ratio = 0.95
        reasons.append("sparse_near_obstacle")
    elif laser_no_return and fwd_total < cfg.sparse_point_threshold * 2:
        glass_suspect = True
        high_cost_ratio = 0.6
        reasons.append("sparse_only")

    state = "SUSPECT" if glass_suspect else "NONE"

    return {
        "state": state,
        "glass_suspect": glass_suspect,
        "glass_confirmed": False,
        "near_obstacle": near_obstacle,
        "high_cost_ratio": round(high_cost_ratio, 3),
        "reasons": reasons,
    }


def detect_tactile_bump(
    *,
    suspect_recent: bool,
    max_tau_nm: float,
    prev_max_tau_nm: float,
    front_max_tau_nm: float,
    imu_horiz_acc: float,
    baseline_imu_horiz: float,
    cfg: TactileConfig,
) -> bool:
    """怀疑态下关节力矩/IMU 冲击 → 触觉确认（零先验碰壁）."""
    if not suspect_recent:
        return False

    tau_spike = (max_tau_nm - prev_max_tau_nm) >= cfg.tau_delta_threshold_nm
    front_hit = front_max_tau_nm >= cfg.tau_front_min_nm
    imu_hit = abs(imu_horiz_acc - baseline_imu_horiz) >= cfg.imu_impact_threshold

    return (tau_spike and front_hit) or (front_hit and imu_hit)


def estimate_obstacle_distance_m(
    *,
    min_range_obstacle_m: Optional[float],
    forward_nearest_m: Optional[float],
    default_m: float = 0.45,
) -> float:
    candidates = [default_m]
    if min_range_obstacle_m is not None and min_range_obstacle_m > 0.05:
        candidates.append(float(min_range_obstacle_m))
    if forward_nearest_m is not None and forward_nearest_m > 0.05:
        candidates.append(float(forward_nearest_m))
    return min(candidates)


def build_glass_wall_patch(
    *,
    robot_x: float,
    robot_y: float,
    robot_z: float,
    yaw: float,
    distance_m: float,
    width_m: float = 0.6,
    height_m: float = 1.2,
    z_base_m: float = 0.15,
    resolution_m: float = 0.05,
) -> List[Tuple[float, float, float]]:
    """在前方 distance 处生成垂直玻璃面点云（world/map 系）."""
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    center_x = robot_x + distance_m * cos_y
    center_y = robot_y + distance_m * sin_y

    # 墙沿 robot 左右方向展开
    lat_x = -sin_y
    lat_y = cos_y

    points: List[Tuple[float, float, float]] = []
    half_w = width_m / 2.0
    w_steps = max(1, int(math.ceil(width_m / resolution_m)))
    z_steps = max(1, int(math.ceil(height_m / resolution_m)))

    for iw in range(w_steps + 1):
        offset = -half_w + (width_m * iw / w_steps)
        px = center_x + offset * lat_x
        py = center_y + offset * lat_y
        for iz in range(z_steps + 1):
            pz = robot_z + z_base_m + (height_m * iz / z_steps)
            points.append((px, py, pz))
    return points
