#include "motionslam_runtime/task_runtime.hpp"

#include <utility>

namespace motionslam::runtime {

bool TaskRuntime::is_terminal(TaskStatus status) noexcept {
  return status == TaskStatus::kSucceeded || status == TaskStatus::kFailed ||
         status == TaskStatus::kCanceled;
}

bool TaskRuntime::is_valid(const TaskIntent& intent) noexcept {
  if (intent.task_id.empty() || intent.task_type.empty() ||
      intent.world_reference.asset_id.empty() ||
      intent.world_reference.frame_id.empty() || intent.skill_sequence.empty()) {
    return false;
  }
  for (const auto& call : intent.skill_sequence) {
    if (call.skill.empty()) {
      return false;
    }
  }
  return true;
}

SubmitResult TaskRuntime::submit(TaskIntent intent) {
  if (!is_valid(intent)) {
    return SubmitResult::kInvalidIntent;
  }
  if (seen_task_ids_.contains(intent.task_id)) {
    return SubmitResult::kDuplicateTaskId;
  }
  if (safety_mode_ != SafetyMode::kNormal) {
    return SubmitResult::kSafetyInterlock;
  }
  if (active_task_ && !is_terminal(active_task_->status)) {
    return SubmitResult::kTaskAlreadyActive;
  }

  seen_task_ids_.insert(intent.task_id);
  active_task_ = TaskSnapshot{std::move(intent), TaskStatus::kAccepted, 0, {}};
  emit(RuntimeEventType::kTaskAccepted);
  return SubmitResult::kAccepted;
}

bool TaskRuntime::start() {
  if (!active_task_ || active_task_->status != TaskStatus::kAccepted ||
      safety_mode_ != SafetyMode::kNormal) {
    return false;
  }

  active_task_->status = TaskStatus::kRunning;
  emit(RuntimeEventType::kTaskStarted);
  emit(RuntimeEventType::kSkillStarted,
       active_task_->intent.skill_sequence.front().skill);
  return true;
}

bool TaskRuntime::pause() {
  if (!active_task_ || active_task_->status != TaskStatus::kRunning) {
    return false;
  }
  active_task_->status = TaskStatus::kPaused;
  emit(RuntimeEventType::kTaskPaused);
  return true;
}

bool TaskRuntime::resume() {
  if (!active_task_ ||
      (active_task_->status != TaskStatus::kPaused &&
       active_task_->status != TaskStatus::kHolding) ||
      safety_mode_ != SafetyMode::kNormal) {
    return false;
  }
  active_task_->status = TaskStatus::kRunning;
  emit(RuntimeEventType::kTaskResumed);
  return true;
}

bool TaskRuntime::cancel() {
  if (!active_task_ || is_terminal(active_task_->status)) {
    return false;
  }
  active_task_->status = TaskStatus::kCanceled;
  emit(RuntimeEventType::kTaskCanceled);
  return true;
}

bool TaskRuntime::fail(std::string reason) {
  if (!active_task_ || is_terminal(active_task_->status) || reason.empty()) {
    return false;
  }
  active_task_->status = TaskStatus::kFailed;
  active_task_->failure_reason = std::move(reason);
  emit(RuntimeEventType::kTaskFailed, {}, active_task_->failure_reason);
  return true;
}

bool TaskRuntime::complete_current_skill() {
  if (!active_task_ || active_task_->status != TaskStatus::kRunning ||
      safety_mode_ != SafetyMode::kNormal) {
    return false;
  }

  const auto& skills = active_task_->intent.skill_sequence;
  const std::string completed = skills[active_task_->current_skill_index].skill;
  emit(RuntimeEventType::kSkillCompleted, completed);
  ++active_task_->current_skill_index;

  if (active_task_->current_skill_index == skills.size()) {
    active_task_->status = TaskStatus::kSucceeded;
    emit(RuntimeEventType::kTaskSucceeded);
    return true;
  }

  emit(RuntimeEventType::kSkillStarted,
       skills[active_task_->current_skill_index].skill);
  return true;
}

void TaskRuntime::set_safety_mode(SafetyMode mode) {
  safety_mode_ = mode;
  if (mode != SafetyMode::kNormal && active_task_ &&
      active_task_->status == TaskStatus::kRunning) {
    active_task_->status = TaskStatus::kHolding;
    const auto reason = mode == SafetyMode::kHold ? "safety_hold"
                                                  : "degraded_not_validated";
    emit(RuntimeEventType::kSafetyHold, {}, reason);
  }
}

void TaskRuntime::emit(RuntimeEventType type, std::string skill,
                       std::string reason) {
  if (!active_task_) {
    return;
  }
  events_.push_back(
      RuntimeEvent{type, active_task_->intent.task_id, std::move(skill),
                   std::move(reason)});
}

}  // namespace motionslam::runtime