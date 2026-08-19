"""thing_envelope / behavior_mode 校验单测."""
from __future__ import annotations

import copy
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from behavior_mode import (  # noqa: E402
    extract_nav_goal,
    flatten_identity,
    validate_thing_envelope,
)
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
        local_robot_id="go2_001",
        local_platform="unitree_go2_edu",
    )
    for k, v in kwargs.items():
        setattr(base, k, v)
    return base


def _execute_envelope(now: int | None = None) -> dict:
    ts = now if now is not None else time.time_ns()
    return {
        "schema": "motionslam.thing_envelope.v1",
        "tid": "cmd_mode_001",
        "bid": "demo_live",
        "timestamp": ts,
        "method": "behavior_mode.execute",
        "data": {
            "profile": {
                "sn": "go2_001",
                "platform": "unitree_go2_edu",
                "domain": "legged",
                "type": "go2",
                "thing_version": "1.0.0",
                "capability_tags": ["Navigate", "ReturnHome", "RecoverGlass"],
            },
            "properties": {
                "scene_type": "intersection",
                "semantic_tags": ["turn_right_at_next_junction"],
                "language_query": "前面路口右转",
                "map_id": "demo_live_map",
                "floor_id": "floor_01",
                "frame_id": "world",
                "context_generation": 2,
                "mode_state": "INIT",
            },
            "services": {
                "actions": [
                    {
                        "skill": "Navigate",
                        "goal": {
                            "type": "waypoint",
                            "x": 2.0,
                            "y": 0.5,
                            "z": 0.35,
                            "yaw_rad": 0.0,
                        },
                    }
                ]
            },
            "policy": {
                "mode_id": "navigate_intersection_cautious",
                "mode_version": "1",
                "policy_ref": "policydb://scene/intersection/cautious",
                "lifecycle_policy": {
                    "on_link_lost": "ReturnHome",
                    "on_glass_trap": "RecoverGlass",
                },
                "adaptation_rationale": "路口狭窄且右侧有玻璃",
            },
            "models": {
                "world_model": {"name": "edge_vla", "version": "demo"},
                "planner": {"name": "pct_resident", "version": "cpp"},
                "controller": {"name": "scan_navi_mode_3"},
            },
        },
    }


def test_accept_execute_envelope():
    res = validate_thing_envelope(_execute_envelope(), _ctx())
    assert res.ok, res.reason
    assert res.reason == "ACCEPT"


def test_flatten_identity_maps_tid_bid():
    ident = flatten_identity(_execute_envelope())
    assert ident["command_id"] == "cmd_mode_001"
    assert ident["session_id"] == "demo_live"
    assert ident["map_id"] == "demo_live_map"


def test_extract_nav_goal():
    goal = extract_nav_goal(_execute_envelope())
    assert goal is not None
    assert goal["x"] == 2.0


def test_reject_missing_scene():
    raw = _execute_envelope()
    del raw["data"]["properties"]["scene_type"]
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_MISSING_SCENE"


def test_reject_missing_robot():
    raw = _execute_envelope()
    del raw["data"]["profile"]
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_MISSING_ROBOT"


def test_reject_robot_mismatch():
    res = validate_thing_envelope(_execute_envelope(), _ctx(local_robot_id="go2_999"))
    assert not res.ok
    assert res.reason == "REJECT_ROBOT_MISMATCH"


def test_reject_unknown_mode():
    raw = _execute_envelope()
    raw["data"]["policy"]["mode_id"] = "nav2_smac"
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason.startswith("REJECT_MODE_UNKNOWN")


def test_reject_forbidden_cmd_vel():
    raw = _execute_envelope()
    raw["data"]["services"]["twist"] = {"linear": 0.2}
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason.startswith("REJECT_FORBIDDEN_CONTROL")


def test_reject_events_on_downlink():
    raw = _execute_envelope()
    raw["data"]["events"] = [{"event": "mode_exception", "reason": "glass_trap"}]
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_EVENTS_ON_SERVICE"


def test_reject_unresolved_semantic_ref():
    raw = _execute_envelope()
    raw["data"]["services"]["actions"][0]["goal"] = {
        "type": "semantic_ref",
        "ref": "next_intersection_exit_right",
    }
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_UNRESOLVED_GOAL"


def test_accept_semantic_ref_with_waypoint():
    raw = _execute_envelope()
    raw["data"]["services"]["actions"][0]["goal"] = {
        "type": "semantic_ref",
        "ref": "next_intersection_exit_right",
        "x": 2.0,
        "y": 0.5,
        "z": 0.35,
    }
    res = validate_thing_envelope(raw, _ctx())
    assert res.ok, res.reason


def test_reject_missing_models():
    raw = _execute_envelope()
    del raw["data"]["models"]
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_MISSING_MODELS"


def test_accept_progress_exception_with_fragments():
    raw = copy.deepcopy(_execute_envelope())
    raw["method"] = "behavior_mode.progress"
    raw["data"]["properties"]["mode_state"] = "EXCEPTION"
    raw["data"].pop("services", None)
    raw["data"]["events"] = [
        {"event": "mode_exception", "reason": "glass_trap", "need_reply": 1}
    ]
    raw["data"]["world_fragments"] = [
        {
            "layer_id": "topology_subgraph",
            "context_generation": 2,
            "ref": "tcp://127.0.0.1:9877",
            "digest": "abc",
        }
    ]
    res = validate_thing_envelope(raw, _ctx())
    assert res.ok, res.reason


def test_reject_inline_fragment_payload():
    raw = copy.deepcopy(_execute_envelope())
    raw["method"] = "behavior_mode.progress"
    raw["data"]["properties"]["mode_state"] = "ACTIVE"
    raw["data"].pop("services", None)
    raw["data"]["events"] = []
    raw["data"]["world_fragments"] = [
        {"layer_id": "topology_subgraph", "ref": "x", "points": [1, 2, 3]}
    ]
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert "INLINE_PAYLOAD" in res.reason


def test_reject_wrong_schema():
    raw = _execute_envelope()
    raw["schema"] = "motionslam.action_group.v1"
    res = validate_thing_envelope(raw, _ctx())
    assert not res.ok
    assert res.reason.startswith("REJECT_SCHEMA")
