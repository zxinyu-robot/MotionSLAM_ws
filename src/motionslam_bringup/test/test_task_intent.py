"""task_intent 纯逻辑单元测试（无 ROS）。"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from task_intent import (  # noqa: E402
    SCHEMA_INTENT,
    build_feedback,
    validate_task_intent,
)


def _intent(**over):
    msg = {
        "schema": SCHEMA_INTENT,
        "task_id": "t-1",
        "robot": {"platform": "unitree_go2_edu", "sn": "go2_001"},
        "world_ref": {"map_id": "demo_live_map", "floor_id": "f1",
                      "frame_id": "world", "revision": 3},
        "skills": [{"name": "Navigate", "goal": {"x": 2.0, "y": 0.5, "z": 0.35}}],
        "policy": {"deadline_s": 120.0},
    }
    msg.update(over)
    return msg


class TestValidate:
    def test_valid(self):
        intent, err = validate_task_intent(_intent())
        assert err is None and intent is not None
        assert intent.task_id == "t-1"
        assert intent.revision == 3
        assert intent.skills[0].goal.x == 2.0
        assert intent.deadline_s == 120.0

    def test_reject_schema(self):
        _, err = validate_task_intent(_intent(schema="motionslam.thing_envelope.v1"))
        assert err.startswith("REJECT_SCHEMA")

    def test_reject_forbidden_control(self):
        bad = _intent()
        bad["skills"][0]["cmd_vel"] = {"vx": 1.0}
        _, err = validate_task_intent(bad)
        assert err.startswith("REJECT_FORBIDDEN_CONTROL")

    def test_reject_unsupported_skill(self):
        _, err = validate_task_intent(
            _intent(skills=[{"name": "FlyTo", "goal": {"x": 0, "y": 0, "z": 0}}])
        )
        assert err.startswith("REJECT_SKILL_UNSUPPORTED")

    def test_reject_removed_glass_skill(self):
        _, err = validate_task_intent(_intent(skills=[{"name": "RecoverGlass"}]))
        assert err.startswith("REJECT_SKILL_UNSUPPORTED")

    def test_reject_missing_goal(self):
        _, err = validate_task_intent(_intent(skills=[{"name": "Navigate"}]))
        assert "MISSING_GOAL" in err

    def test_capability_narrowing(self):
        _, err = validate_task_intent(
            _intent(skills=[{"name": "ExploreFrontier"}]),
            allowed_skills={"Navigate", "ReturnHome"},
        )
        assert err.startswith("REJECT_SKILL_UNSUPPORTED")

    def test_reject_bad_revision(self):
        _, err = validate_task_intent(_intent(world_ref={"map_id": "m"}))
        assert err == "REJECT_BAD_REVISION"

    def test_return_home_no_goal(self):
        intent, err = validate_task_intent(
            _intent(skills=[{"name": "Navigate", "goal": {"x": 1, "y": 1, "z": 0.35}},
                            {"name": "ReturnHome"}])
        )
        assert err is None and len(intent.skills) == 2

    def test_feedback_shape(self):
        fb = build_feedback(task_id="t", state="running", event="subgoal_reached",
                            progress={"i": 1}, timestamp_ns=1)
        assert fb["schema"] == "motionslam.execution_feedback.v1"
        assert fb["task_id"] == "t"
