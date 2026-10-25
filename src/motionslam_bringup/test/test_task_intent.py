"""task_intent 纯逻辑单元测试（无 ROS）。"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from task_intent import (  # noqa: E402
    SCHEMA_INTENT,
    AdmitDecision,
    TaskAdmission,
    check_forbidden_areas,
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


class TestExecutionAndAreas:
    def test_defaults_not_preemptible(self):
        intent, err = validate_task_intent(_intent())
        assert err is None
        assert intent.priority == 0 and intent.preemptible is False
        assert intent.forbidden_areas == []

    def test_parse_execution_navigation(self):
        intent, err = validate_task_intent(
            _intent(execution={"priority": 3, "preemptible": True},
                    navigation={"forbidden_areas": ["restricted_C"]})
        )
        assert err is None
        assert intent.priority == 3 and intent.preemptible is True
        assert intent.forbidden_areas == ["restricted_C"]

    @pytest.mark.parametrize("execution,code", [
        ({"priority": "high"}, "REJECT_BAD_PRIORITY"),
        ({"preemptible": "yes"}, "REJECT_BAD_PREEMPTIBLE"),
    ])
    def test_reject_bad_execution(self, execution, code):
        _, err = validate_task_intent(_intent(execution=execution))
        assert err == code

    @pytest.mark.parametrize("areas", ["restricted_C", [""], [1]])
    def test_reject_bad_forbidden_areas(self, areas):
        _, err = validate_task_intent(_intent(navigation={"forbidden_areas": areas}))
        assert err == "REJECT_BAD_FORBIDDEN_AREAS"

    SQUARE = {"restricted_C": [(1.0, 0.0), (3.0, 0.0), (3.0, 1.0), (1.0, 1.0)]}

    def test_goal_inside_forbidden_area_rejected(self):
        intent, _ = validate_task_intent(_intent(navigation={"forbidden_areas": ["restricted_C"]}))
        err = check_forbidden_areas(intent, self.SQUARE)  # goal (2.0, 0.5)
        assert err == "REJECT_GOAL_IN_FORBIDDEN_AREA:restricted_C@skill0"

    def test_goal_outside_forbidden_area_ok(self):
        intent, _ = validate_task_intent(_intent(
            skills=[{"name": "Navigate", "goal": {"x": 5.0, "y": 0.5, "z": 0.0}}],
            navigation={"forbidden_areas": ["restricted_C"]},
        ))
        assert check_forbidden_areas(intent, self.SQUARE) is None

    def test_unknown_area_fails_closed(self):
        intent, _ = validate_task_intent(_intent(navigation={"forbidden_areas": ["zone_X"]}))
        assert check_forbidden_areas(intent, self.SQUARE) == "REJECT_UNKNOWN_AREA:zone_X"
        assert check_forbidden_areas(intent, {}) == "REJECT_UNKNOWN_AREA:zone_X"


def _parsed(**over):
    intent, err = validate_task_intent(_intent(**over))
    assert err is None
    return intent


class TestAdmission:
    def test_accept_then_duplicate_after_finish(self):
        adm = TaskAdmission()
        t1 = _parsed(task_id="t-1")
        assert adm.decide(t1, None) == AdmitDecision.ACCEPT
        adm.record(t1.task_id)
        assert adm.decide(t1, None) == "REJECT_DUPLICATE_TASK_ID"

    def test_same_id_while_active_is_idempotent(self):
        adm = TaskAdmission()
        t1 = _parsed(task_id="t-1")
        adm.record(t1.task_id)
        assert adm.decide(_parsed(task_id="t-1"), t1) == AdmitDecision.IDEMPOTENT

    def test_busy_when_not_preemptible(self):
        adm = TaskAdmission()
        active = _parsed(task_id="t-1", execution={"priority": 0})
        new = _parsed(task_id="t-2", execution={"priority": 9})
        assert adm.decide(new, active) == "REJECT_BUSY"

    def test_busy_when_priority_not_higher(self):
        adm = TaskAdmission()
        active = _parsed(task_id="t-1", execution={"priority": 3, "preemptible": True})
        new = _parsed(task_id="t-2", execution={"priority": 3})
        assert adm.decide(new, active) == "REJECT_BUSY"

    def test_preempt_higher_priority(self):
        adm = TaskAdmission()
        active = _parsed(task_id="t-1", execution={"priority": 1, "preemptible": True})
        new = _parsed(task_id="t-2", execution={"priority": 2})
        assert adm.decide(new, active) == AdmitDecision.PREEMPT

    def test_history_bounded(self):
        adm = TaskAdmission(history=2)
        for tid in ("a", "b", "c"):
            adm.record(tid)
        assert adm.decide(_parsed(task_id="a"), None) == AdmitDecision.ACCEPT
        assert adm.decide(_parsed(task_id="c"), None) == "REJECT_DUPLICATE_TASK_ID"
