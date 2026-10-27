# -*- coding: utf-8 -*-
"""task_runtime 的 Python 适配层（与 C++ TaskRuntime 同契约，过渡形态）。

项目面向工程落地，编排/运行时/恢复机制的权威实现在 C++ 侧
（runtime/task_runtime）。本适配层仅用于云端验证"任务意图→生命周期→
终态反馈"入口链路，字段与状态机严格对齐
runtime/task_runtime/include/motionslam_runtime/task_runtime.hpp；
工程形态迁移后由 C++ TaskRuntime 接管，调用方接口不变。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class TaskStatus(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    PAUSED = "paused"
    HOLDING = "holding"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


class SafetyMode(str, Enum):
    NORMAL = "normal"
    DEGRADED = "degraded"
    HOLD = "hold"


class SubmitResult(str, Enum):
    ACCEPTED = "accepted"
    INVALID_INTENT = "invalid_intent"
    DUPLICATE_TASK_ID = "duplicate_task_id"
    TASK_ALREADY_ACTIVE = "task_already_active"
    SAFETY_INTERLOCK = "safety_interlock"


@dataclass
class WorldReference:
    asset_id: str = ""
    revision: int = 0
    context_generation: int = 0
    frame_id: str = ""


@dataclass
class SkillCall:
    skill: str
    target: str = ""


@dataclass
class ExecutionPolicy:
    priority: int = 0
    deadline_ns: int = 0
    preemptible: bool = False


@dataclass
class TaskIntent:
    task_id: str
    task_type: str
    goal: str
    target_area: str = ""
    world_reference: WorldReference = field(default_factory=WorldReference)
    skill_sequence: list[SkillCall] = field(default_factory=list)
    execution: ExecutionPolicy = field(default_factory=ExecutionPolicy)


@dataclass
class TaskSnapshot:
    intent: TaskIntent
    status: TaskStatus = TaskStatus.ACCEPTED
    current_skill_index: int = 0
    failure_reason: str = ""


_TERMINAL = {TaskStatus.SUCCEEDED, TaskStatus.FAILED, TaskStatus.CANCELED}


def _is_valid(intent: TaskIntent) -> bool:
    return bool(intent.task_id and intent.task_type and intent.goal)


class TaskRuntimeAdapter:
    """单活动任务的生命周期机，语义同 C++ TaskRuntime。"""

    def __init__(self) -> None:
        self._safety_mode = SafetyMode.NORMAL
        self._active: Optional[TaskSnapshot] = None
        self._seen_task_ids: set[str] = set()
        self.events: list[tuple[str, str]] = []

    @property
    def safety_mode(self) -> SafetyMode:
        return self._safety_mode

    def set_safety_mode(self, mode: SafetyMode) -> None:
        prev = self._safety_mode
        self._safety_mode = mode
        if mode == SafetyMode.HOLD and self._active and self._active.status == TaskStatus.RUNNING:
            self._active.status = TaskStatus.HOLDING
            self.events.append(("safety_hold", self._active.intent.task_id))
        elif prev == SafetyMode.HOLD and mode != SafetyMode.HOLD \
                and self._active and self._active.status == TaskStatus.HOLDING:
            self._active.status = TaskStatus.RUNNING
            self.events.append(("task_resumed", self._active.intent.task_id))

    @property
    def active_task(self) -> Optional[TaskSnapshot]:
        return self._active

    def submit(self, intent: TaskIntent) -> SubmitResult:
        if not _is_valid(intent):
            return SubmitResult.INVALID_INTENT
        if intent.task_id in self._seen_task_ids:
            return SubmitResult.DUPLICATE_TASK_ID
        if self._safety_mode == SafetyMode.HOLD:
            return SubmitResult.SAFETY_INTERLOCK
        if self._active and self._active.status not in _TERMINAL:
            return SubmitResult.TASK_ALREADY_ACTIVE
        self._active = TaskSnapshot(intent=intent)
        self._seen_task_ids.add(intent.task_id)
        self.events.append(("task_accepted", intent.task_id))
        return SubmitResult.ACCEPTED

    def start(self) -> bool:
        if not self._active or self._active.status != TaskStatus.ACCEPTED:
            return False
        self._active.status = (
            TaskStatus.HOLDING if self._safety_mode == SafetyMode.HOLD else TaskStatus.RUNNING
        )
        self.events.append(("task_started", self._active.intent.task_id))
        return True

    def pause(self) -> bool:
        if self._active and self._active.status == TaskStatus.RUNNING:
            self._active.status = TaskStatus.PAUSED
            self.events.append(("task_paused", self._active.intent.task_id))
            return True
        return False

    def resume(self) -> bool:
        if self._active and self._active.status == TaskStatus.PAUSED:
            self._active.status = TaskStatus.RUNNING
            self.events.append(("task_resumed", self._active.intent.task_id))
            return True
        return False

    def cancel(self) -> bool:
        if self._active and self._active.status not in _TERMINAL:
            self._active.status = TaskStatus.CANCELED
            self.events.append(("task_canceled", self._active.intent.task_id))
            return True
        return False

    def fail(self, reason: str = "") -> bool:
        if self._active and self._active.status not in _TERMINAL:
            self._active.status = TaskStatus.FAILED
            self._active.failure_reason = reason
            self.events.append(("task_failed", self._active.intent.task_id))
            return True
        return False

    def complete_current_skill(self) -> bool:
        if not self._active or self._active.status not in (
            TaskStatus.RUNNING, TaskStatus.HOLDING,
        ):
            return False
        snap = self._active
        skills = snap.intent.skill_sequence
        if skills:
            self.events.append(
                ("skill_completed", skills[snap.current_skill_index].skill))
        snap.current_skill_index += 1
        if snap.current_skill_index >= max(1, len(skills)):
            snap.status = TaskStatus.SUCCEEDED
            self.events.append(("task_succeeded", snap.intent.task_id))
        return True
