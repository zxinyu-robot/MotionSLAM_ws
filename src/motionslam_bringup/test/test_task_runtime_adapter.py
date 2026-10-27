# -*- coding: utf-8 -*-
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from task_runtime_adapter import (  # noqa: E402
    SkillCall, SubmitResult, SafetyMode, TaskIntent, TaskRuntimeAdapter,
    TaskStatus,
)


def _intent(task_id="t1", skills=None):
    return TaskIntent(
        task_id=task_id, task_type="navigate", goal="room_a",
        skill_sequence=skills or [SkillCall("navigate", "room_a")],
    )


def test_submit_start_complete_succeeds():
    rt = TaskRuntimeAdapter()
    assert rt.submit(_intent()) == SubmitResult.ACCEPTED
    assert rt.start()
    assert rt.complete_current_skill()
    assert rt.active_task.status == TaskStatus.SUCCEEDED


def test_invalid_intent_rejected():
    rt = TaskRuntimeAdapter()
    assert rt.submit(TaskIntent(task_id="", task_type="nav", goal="g")) == \
        SubmitResult.INVALID_INTENT


def test_duplicate_task_id_rejected():
    rt = TaskRuntimeAdapter()
    rt.submit(_intent("t1"))
    rt.start()
    assert rt.submit(_intent("t1")) == SubmitResult.DUPLICATE_TASK_ID


def test_second_task_while_active_rejected():
    rt = TaskRuntimeAdapter()
    rt.submit(_intent("t1"))
    rt.start()
    assert rt.submit(_intent("t2")) == SubmitResult.TASK_ALREADY_ACTIVE


def test_safety_hold_blocks_and_resume():
    rt = TaskRuntimeAdapter()
    rt.set_safety_mode(SafetyMode.HOLD)
    assert rt.submit(_intent()) == SubmitResult.SAFETY_INTERLOCK
    rt.set_safety_mode(SafetyMode.NORMAL)
    assert rt.submit(_intent()) == SubmitResult.ACCEPTED
    rt.start()
    rt.set_safety_mode(SafetyMode.HOLD)
    assert rt.active_task.status == TaskStatus.HOLDING
    rt.set_safety_mode(SafetyMode.NORMAL)
    assert rt.active_task.status == TaskStatus.RUNNING


def test_pause_resume_cancel():
    rt = TaskRuntimeAdapter()
    rt.submit(_intent())
    rt.start()
    assert rt.pause()
    assert rt.active_task.status == TaskStatus.PAUSED
    assert rt.resume()
    assert rt.cancel()
    assert rt.active_task.status == TaskStatus.CANCELED


def test_multi_skill_sequence():
    rt = TaskRuntimeAdapter()
    rt.submit(_intent(skills=[SkillCall("a"), SkillCall("b")]))
    rt.start()
    rt.complete_current_skill()
    assert rt.active_task.status == TaskStatus.RUNNING
    rt.complete_current_skill()
    assert rt.active_task.status == TaskStatus.SUCCEEDED
    kinds = [e[0] for e in rt.events]
    assert kinds == ["task_accepted", "task_started",
                     "skill_completed", "skill_completed", "task_succeeded"]
