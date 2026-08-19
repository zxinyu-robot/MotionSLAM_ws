"""semantic_directive / task_context 校验单测."""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from semantic_directive import validate_semantic_directive  # noqa: E402
from task_context import TaskIdentityContext  # noqa: E402


def _ctx(**kwargs) -> TaskIdentityContext:
    base = TaskIdentityContext(
        local_session_id="demo_live",
        local_floor_id="floor_01",
        local_map_id="demo_live_map",
        local_frame_id="world",
        local_map_version=10,
        context_generation=3,
        known_frontier_ids={"bt_next_subgoal"},
    )
    for k, v in kwargs.items():
        setattr(base, k, v)
    return base


def test_accept_valid_directive():
    now = time.time_ns()
    raw = {
        "schema": "motionslam.semantic_directive.v1",
        "session_id": "demo_live",
        "floor_id": "floor_01",
        "map_id": "demo_live_map",
        "frame_id": "world",
        "map_version": 5,
        "context_generation": 2,
        "issued_at_ns": now,
        "ttl_ms": 5000,
        "language_query": "找红色灭火器",
        "target_labels": ["fire_extinguisher"],
        "search_mode": "explore",
        "candidate_frontier_ids": ["bt_next_subgoal"],
    }
    res = validate_semantic_directive(raw, _ctx())
    assert res.ok
    assert res.reason == "ACCEPT"


def test_reject_expired():
    raw = {
        "schema": "motionslam.semantic_directive.v1",
        "session_id": "demo_live",
        "issued_at_ns": time.time_ns() - 20_000_000_000,
        "ttl_ms": 1000,
    }
    res = validate_semantic_directive(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_EXPIRED"


def test_reject_version():
    raw = {
        "schema": "motionslam.semantic_directive.v1",
        "session_id": "demo_live",
        "map_version": 99,
        "ttl_ms": 5000,
        "issued_at_ns": time.time_ns(),
    }
    res = validate_semantic_directive(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_VERSION"


def test_reject_session():
    raw = {
        "schema": "motionslam.semantic_directive.v1",
        "session_id": "wrong",
        "issued_at_ns": time.time_ns(),
        "ttl_ms": 5000,
    }
    res = validate_semantic_directive(raw, _ctx())
    assert not res.ok
    assert res.reason == "REJECT_SESSION"


def test_reject_context_generation():
    raw = {
        "schema": "motionslam.semantic_directive.v1",
        "session_id": "demo_live",
        "context_generation": 99,
        "issued_at_ns": time.time_ns(),
        "ttl_ms": 5000,
    }
    res = validate_semantic_directive(raw, _ctx(context_generation=3))
    assert not res.ok
    assert res.reason == "REJECT_CONTEXT"


def test_reject_bad_search_mode():
    raw = {
        "schema": "motionslam.semantic_directive.v1",
        "session_id": "demo_live",
        "search_mode": "fly",
        "issued_at_ns": time.time_ns(),
        "ttl_ms": 5000,
    }
    res = validate_semantic_directive(raw, _ctx())
    assert not res.ok
    assert "REJECT_SEARCH_MODE" in res.reason
