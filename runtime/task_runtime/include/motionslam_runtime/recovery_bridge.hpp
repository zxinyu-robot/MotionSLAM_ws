#pragma once

#include <cstdint>
#include <string>

#include "legged_navigation/recovery_plugin.h"
#include "motionslam_runtime/plugin_registry.hpp"
#include "motionslam_runtime/task_runtime.hpp"

namespace motionslam::runtime {

// Capability strings shared by the recovery plugin and its consumers.
inline constexpr const char* kCapLocalizationHealth = "localization_health";
inline constexpr const char* kCapExecutionAuthorityAdvice =
    "execution_authority_advice";
inline constexpr const char* kCapRecoveryActionAdvice = "recovery_action_advice";

// Registry descriptor for LNR RecoveryPlugin: kind=Recovery, authority=Advice.
PluginDescriptor lnr_recovery_descriptor(std::int64_t health_interval_ns);

// What the plugin hands to the BT hub. Advice only; the BT decides.
struct RecoveryAdvice {
  std::string plugin_id;
  legged_navigation::RecoveryAuthority authority{
      legged_navigation::RecoveryAuthority::Full};
  legged_navigation::RecoveryReason reason{
      legged_navigation::RecoveryReason::None};
  legged_navigation::RecoveryAction action{
      legged_navigation::RecoveryAction::None};
  legged_navigation::RecoveryState state{
      legged_navigation::RecoveryState::Tracking};
  std::int64_t stamp_ns{0};
};

// Owns the LNR plugin instance, registers it, and reports its health on every
// update so the registry marks it stale when updates stop.
class RecoveryPluginHost final {
 public:
  explicit RecoveryPluginHost(PluginRegistry& registry,
                              legged_navigation::RecoveryConfig config = {},
                              std::int64_t health_interval_ns = 200'000'000);

  RegisterResult attach();
  bool detach();
  bool attached() const noexcept { return attached_; }

  RecoveryAdvice update(const legged_navigation::RecoveryInput& input,
                        std::int64_t now_ns);

 private:
  PluginRegistry& registry_;
  legged_navigation::RecoveryPlugin plugin_;
  PluginDescriptor descriptor_;
  bool attached_{false};
};

enum class ArbiterDecision {
  kApplied,        // advice changed the runtime state
  kNoChange,       // advice accepted, nothing to do
  kRejected,       // source is not a registered Advice-level recovery plugin
  kStaleFailSafe,  // plugin stale: BT held the task on its own authority
};

// BT-hub side of the boundary. Execution authority stays here: the arbiter
// maps recovery advice onto TaskRuntime safety modes and only undoes holds it
// placed itself.
//   Full       -> Normal; resume a recovery-held task on ResumeNavigation
//   Restricted -> Degraded (fails closed until degraded mode is validated)
//   Suspended  -> Hold
//   Revoked    -> Hold + fail the active task with the reason code
//   stale      -> Hold ("recovery_plugin_stale")
class RecoveryArbiter final {
 public:
  ArbiterDecision apply(const PluginRegistry& registry,
                        const RecoveryAdvice& advice, TaskRuntime& runtime,
                        std::int64_t now_ns);

  bool holding() const noexcept { return held_by_recovery_; }
  const std::string& last_reason() const noexcept { return last_reason_; }

 private:
  static bool set_mode(TaskRuntime& runtime, SafetyMode mode);

  bool held_by_recovery_{false};
  std::string last_reason_;
};

}  // namespace motionslam::runtime
