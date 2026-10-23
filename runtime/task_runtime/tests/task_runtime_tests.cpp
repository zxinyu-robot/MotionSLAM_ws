#include "motionslam_runtime/task_runtime.hpp"

#include <cassert>

using motionslam::runtime::ExecutionPolicy;
using motionslam::runtime::RuntimeEventType;
using motionslam::runtime::SafetyMode;
using motionslam::runtime::SkillCall;
using motionslam::runtime::SubmitResult;
using motionslam::runtime::TaskIntent;
using motionslam::runtime::TaskRuntime;
using motionslam::runtime::TaskStatus;
using motionslam::runtime::WorldReference;

namespace {

TaskIntent make_task(const char* task_id = "task-1") {
  TaskIntent intent;
  intent.task_id = task_id;
  intent.task_type = "navigation";
  intent.goal = "inspect-area";
  intent.target_area = "zone-a";
  intent.world_reference = WorldReference{"map.site", 7, 3, "world"};
  intent.skill_sequence = {SkillCall{"navigate", "zone-a"},
                           SkillCall{"inspect", "zone-a"}};
  intent.execution = ExecutionPolicy{2, 1000000000, true};
  return intent;
}

void test_sequence_completes_in_order() {
  TaskRuntime runtime;
  assert(runtime.submit(make_task()) == SubmitResult::kAccepted);
  assert(runtime.start());
  assert(runtime.active_task()->status == TaskStatus::kRunning);
  assert(runtime.active_task()->current_skill_index == 0);
  assert(runtime.complete_current_skill());
  assert(runtime.active_task()->current_skill_index == 1);
  assert(runtime.complete_current_skill());
  assert(runtime.active_task()->status == TaskStatus::kSucceeded);
  assert(runtime.events().back().type == RuntimeEventType::kTaskSucceeded);
}

void test_hold_requires_explicit_resume() {
  TaskRuntime runtime;
  assert(runtime.submit(make_task()) == SubmitResult::kAccepted);
  assert(runtime.start());
  runtime.set_safety_mode(SafetyMode::kHold);
  assert(runtime.active_task()->status == TaskStatus::kHolding);
  assert(!runtime.complete_current_skill());
  runtime.set_safety_mode(SafetyMode::kNormal);
  assert(runtime.active_task()->status == TaskStatus::kHolding);
  assert(runtime.resume());
  assert(runtime.active_task()->status == TaskStatus::kRunning);
}

void test_single_active_task_and_idempotency_guard() {
  TaskRuntime runtime;
  assert(runtime.submit(make_task("task-a")) == SubmitResult::kAccepted);
  assert(runtime.submit(make_task("task-b")) ==
         SubmitResult::kTaskAlreadyActive);
  assert(runtime.start());
  assert(runtime.cancel());
  assert(runtime.submit(make_task("task-a")) ==
         SubmitResult::kDuplicateTaskId);
  assert(runtime.submit(make_task("task-b")) == SubmitResult::kAccepted);
}

void test_invalid_task_is_rejected() {
  TaskRuntime runtime;
  TaskIntent invalid;
  invalid.task_id = "missing-world-ref";
  assert(runtime.submit(invalid) == SubmitResult::kInvalidIntent);
  assert(!runtime.active_task());
}

void test_safety_hold_rejects_new_task() {
  TaskRuntime runtime;
  runtime.set_safety_mode(SafetyMode::kHold);
  assert(runtime.submit(make_task()) == SubmitResult::kSafetyInterlock);
  assert(!runtime.active_task());
}

void test_unvalidated_degraded_mode_fails_closed() {
  TaskRuntime runtime;
  assert(runtime.submit(make_task()) == SubmitResult::kAccepted);
  assert(runtime.start());
  runtime.set_safety_mode(SafetyMode::kDegraded);
  assert(runtime.active_task()->status == TaskStatus::kHolding);
  assert(!runtime.resume());
  runtime.set_safety_mode(SafetyMode::kNormal);
  assert(runtime.active_task()->status == TaskStatus::kHolding);
  assert(runtime.resume());
  assert(runtime.active_task()->status == TaskStatus::kRunning);

  TaskRuntime blocked_runtime;
  blocked_runtime.set_safety_mode(SafetyMode::kDegraded);
  assert(blocked_runtime.submit(make_task()) == SubmitResult::kSafetyInterlock);
  assert(!blocked_runtime.active_task());
}

}  // namespace

int main() {
  test_sequence_completes_in_order();
  test_hold_requires_explicit_resume();
  test_single_active_task_and_idempotency_guard();
  test_invalid_task_is_rejected();
  test_safety_hold_rejects_new_task();
  test_unvalidated_degraded_mode_fails_closed();
  return 0;
}