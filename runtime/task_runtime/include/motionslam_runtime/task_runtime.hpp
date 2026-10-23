#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace motionslam::runtime {

struct WorldReference {
  std::string asset_id;
  std::uint64_t revision{0};
  std::uint64_t context_generation{0};
  std::string frame_id;
};

struct SkillCall {
  std::string skill;
  std::string target;
};

struct ExecutionPolicy {
  int priority{0};
  std::uint64_t deadline_ns{0};
  bool preemptible{false};
};

struct TaskIntent {
  std::string task_id;
  std::string task_type;
  std::string goal;
  std::string target_area;
  WorldReference world_reference;
  std::vector<SkillCall> skill_sequence;
  ExecutionPolicy execution;
};

enum class TaskStatus {
  kAccepted,
  kRunning,
  kPaused,
  kHolding,
  kSucceeded,
  kFailed,
  kCanceled,
};

enum class SafetyMode {
  kNormal,
  kDegraded,
  kHold,
};

enum class RuntimeEventType {
  kTaskAccepted,
  kTaskStarted,
  kSkillStarted,
  kSkillCompleted,
  kTaskPaused,
  kTaskResumed,
  kSafetyHold,
  kTaskSucceeded,
  kTaskFailed,
  kTaskCanceled,
};

struct RuntimeEvent {
  RuntimeEventType type;
  std::string task_id;
  std::string skill;
  std::string reason;
};

struct TaskSnapshot {
  TaskIntent intent;
  TaskStatus status{TaskStatus::kAccepted};
  std::size_t current_skill_index{0};
  std::string failure_reason;
};

enum class SubmitResult {
  kAccepted,
  kInvalidIntent,
  kDuplicateTaskId,
  kTaskAlreadyActive,
  kSafetyInterlock,
};

class TaskRuntime final {
 public:
  SubmitResult submit(TaskIntent intent);
  bool start();
  bool pause();
  bool resume();
  bool cancel();
  bool fail(std::string reason);
  bool complete_current_skill();

  void set_safety_mode(SafetyMode mode);
  SafetyMode safety_mode() const noexcept { return safety_mode_; }
  const std::optional<TaskSnapshot>& active_task() const noexcept {
    return active_task_;
  }
  const std::vector<RuntimeEvent>& events() const noexcept { return events_; }
  void clear_events() { events_.clear(); }

 private:
  static bool is_terminal(TaskStatus status) noexcept;
  static bool is_valid(const TaskIntent& intent) noexcept;
  void emit(RuntimeEventType type, std::string skill = {},
            std::string reason = {});

  SafetyMode safety_mode_{SafetyMode::kNormal};
  std::optional<TaskSnapshot> active_task_;
  std::set<std::string> seen_task_ids_;
  std::vector<RuntimeEvent> events_;
};

}  // namespace motionslam::runtime