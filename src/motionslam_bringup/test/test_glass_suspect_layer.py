"""glass_suspect_layer 零先验几何/强度分析单元测试."""
import math
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from glass_suspect_analysis import (  # noqa: E402
    GlassAnalysisConfig,
    TactileConfig,
    analyze_forward_rays,
    build_glass_wall_patch,
    detect_tactile_bump,
    estimate_obstacle_distance_m,
    fuse_glass_suspect,
)


def _cfg(**overrides) -> GlassAnalysisConfig:
    base = GlassAnalysisConfig(
        forward_range_m=3.0,
        forward_half_angle_rad=math.radians(30.0),
        near_range_min_m=0.3,
        ray_bin_deg=5.0,
        occlusion_gap_min_m=0.2,
        occlusion_violation_min_bins=2,
    )
    for k, v in overrides.items():
        setattr(base, k, v)
    return base


def test_occlusion_violation_wall_void_wall():
    """同 bin 近处墙 + 远处鬼影 → geometry_suspect."""
    points = []
    # 近墙 @ 1.0m，正前方 bin
    for _ in range(8):
        points.append((1.0, 0.0, 0.8, 50.0))
    # 远处鬼影 @ 2.5m，同方向
    for _ in range(4):
        points.append((2.5, 0.0, 0.8, 40.0))

    stats = analyze_forward_rays(
        points,
        robot_x=0.0,
        robot_y=0.0,
        robot_z=0.0,
        yaw=0.0,
        cfg=_cfg(),
    )
    assert stats["occlusion_violations"] >= 1
    assert stats["geometry_suspect"] is True


def test_no_violation_solid_wall_only():
    """仅近处实心墙，无远处点 → 无遮挡矛盾."""
    points = [(1.0, 0.0, 0.8, 60.0) for _ in range(20)]
    stats = analyze_forward_rays(
        points,
        robot_x=0.0,
        robot_y=0.0,
        robot_z=0.0,
        yaw=0.0,
        cfg=_cfg(),
    )
    assert stats["occlusion_violations"] == 0
    assert stats["geometry_suspect"] is False


def test_specular_high_intensity_isolated():
    """孤点高反射 → intensity_suspect."""
    points = [(1.2, 0.0, 0.9, 220.0)]
    stats = analyze_forward_rays(
        points,
        robot_x=0.0,
        robot_y=0.0,
        robot_z=0.0,
        yaw=0.0,
        cfg=_cfg(specular_score_threshold=0.2),
    )
    assert stats["specular_bins"] >= 1
    assert stats["intensity_suspect"] is True


def test_fuse_sparse_near_obstacle_legacy_path():
    """保留 MVP：稀疏 + sport 近障."""
    ray_stats = {
        "geometry_suspect": False,
        "intensity_suspect": False,
        "laser_no_return": True,
        "forward_points": 5,
    }
    fused = fuse_glass_suspect(ray_stats, near_obstacle=True, cfg=_cfg())
    assert fused["glass_suspect"] is True
    assert fused["state"] == "SUSPECT"
    assert "sparse_near_obstacle" in fused["reasons"]


def test_tactile_bump_when_suspect_recent():
    cfg = TactileConfig(tau_delta_threshold_nm=5.0, tau_front_min_nm=10.0)
    assert detect_tactile_bump(
        suspect_recent=True,
        max_tau_nm=20.0,
        prev_max_tau_nm=10.0,
        front_max_tau_nm=18.0,
        imu_horiz_acc=10.0,
        baseline_imu_horiz=9.8,
        cfg=cfg,
    )


def test_tactile_no_bump_without_suspect():
    cfg = TactileConfig()
    assert not detect_tactile_bump(
        suspect_recent=False,
        max_tau_nm=30.0,
        prev_max_tau_nm=10.0,
        front_max_tau_nm=25.0,
        imu_horiz_acc=15.0,
        baseline_imu_horiz=9.8,
        cfg=cfg,
    )


def test_build_wall_patch_forward():
    pts = build_glass_wall_patch(
        robot_x=0.0,
        robot_y=0.0,
        robot_z=0.0,
        yaw=0.0,
        distance_m=1.0,
        width_m=0.2,
        height_m=0.2,
        resolution_m=0.1,
    )
    assert pts
    xs = [p[0] for p in pts]
    assert min(xs) >= 0.9
    assert max(xs) <= 1.1


def test_estimate_obstacle_distance_prefers_near():
    d = estimate_obstacle_distance_m(
        min_range_obstacle_m=0.42,
        forward_nearest_m=1.2,
        default_m=0.45,
    )
    assert d == 0.42
