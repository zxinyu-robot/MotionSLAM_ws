"""action_group 校验单测."""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from action_group import extract_nav_waypoint, validate_action_group  # noqa: E402
from task_context import TaskIdentityContext  # noqa: E402


def _ctx(**kwargs) -> TaskIdentityContext:
    base = TaskIdentityContext(
        local_session_id="demo_live",
        local_floor_id="floor_01",
        local_map_id="demo_live_map",
        local_frame_id="world",
        local_map_version=10,
        context_generation=3,
        lifecycle_active=True,
        localization_ok=True,
        odom_age_s=0.1,
    )
    for k, v in kwargs.items():
        setattr(base, k, v)
    return base


def _nav_action_group(now: int | None = None) -> dict:
    ts = now if now is not None else time.time_ns()
    return {
        "schema": "motionslam.action_group.v1",
        "command_id": "cmd_test_001",
        "task_type": "OBJNAV",
        "language_query": "找灭火器",
        "session_id": "demo_live",
        "floor_id": "floor_01",
        "map_id": "demo_live_map",
        "frame_id": "world",
        "map_version": 5,
        "context_generation": 2,
        "created_ns": ts,
        "ttl_ms": 30000,
        "stages": [
            {
                "stage_id": "NAV_TO_DESTINATION",
                "stage_type": "NAVIGATE",
                "waypoints": [
                    {"x": 2.0, "y": 0.0, "z": 0.35, "yaw_rad": 0.0},
                ],
            },
            {
                "stage_id": "REPORT_RESULT",
                "stage_type": "REPORT",
                "waypoints": [],
            },
        ],
    }


def test_accept_nav_action_group():
    res = validate_action_group(_nav_action_group(), _ctx())
    assert res.ok
    assert res.reason == "ACCEPT"


def test_reject_missing_command_id():
    ag = _nav_action_group()
    del ag["command_id"]
    res = validate_action_group(ag, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_MISSING_COMMAND_ID"


def test_reject_lifecycle_inactive():
    res = validate_action_group(_nav_action_group(), _ctx(lifecycle_active=False))
    assert not res.ok
    assert res.reason == "REJECT_LIFECYCLE"


def test_reject_pgo_optimizing():
    res = validate_action_group(_nav_action_group(), _ctx(pgo_state="OPTIMIZING"))
    assert not res.ok
    assert res.reason == "REJECT_PGO_OPTIMIZING"


def test_extract_nav_waypoint():
    wp = extract_nav_waypoint(_nav_action_group())
    assert wp is not None
    assert wp["x"] == 2.0
    assert wp["y"] == 0.0
