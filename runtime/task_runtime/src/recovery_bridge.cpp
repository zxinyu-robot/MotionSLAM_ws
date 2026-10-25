#include "motionslam_runtime/recovery_bridge.hpp"

#include <string>

namespace motionslam::runtime {

namespace lnr = legged_navigation;

PluginDescriptor lnr_recovery_descriptor(std::int64_t health_interval_ns) {
  PluginDescriptor d;
  d.plugin_id = std::string(lnr::RecoveryPlugin::plugin_id);
  d.name = "LNR relocalization recovery";
  d.interface_version = std::string(lnr::RecoveryPlugin::interface_version);
  d.kind = PluginKind::kRecovery;
  d.authority = PluginAuthority::kAdvice;
  d.provides = {kCapExecutionAuthorityAdvice, kCapRecoveryActionAdvice};
  d.needs = {kCapLocalizationHealth};
  d.health_interval_ns = health_interval_ns;
  return d;
}

RecoveryPluginHost::RecoveryPluginHost(PluginRegistry& registry,
                                       lnr::RecoveryConfig config,
                                       std::int64_t health_interval_ns)
    : registry_(registry),
      plugin_(config),
      descriptor_(lnr_recovery_descriptor(health_interval_ns)) {}

RegisterResult RecoveryPluginHost::attach() {
  const auto result = registry_.add(descriptor_);
  attached_ = result == RegisterResult::kRegistered;
  return result;
}

bool RecoveryPluginHost::detach() {
  if (!attached_) {
    return false;
  }
  attached_ = false;
  return registry_.remove(descriptor_.plugin_id);
}

RecoveryAdvice RecoveryPluginHost::update(const lnr::RecoveryInput& input,
                                          std::int64_t now_ns) {
  const auto out = plugin_.update(input);
  if (attached_) {
    registry_.report_health(
        descriptor_.plugin_id, true,
        std::string(lnr::RecoveryPlugin::name(out.authority)), now_ns);
  }
  return RecoveryAdvice{descriptor_.plugin_id, out.authority, out.reason,
                        out.action,           out.current,   now_ns};
}

bool RecoveryArbiter::set_mode(TaskRuntime& runtime, SafetyMode mode) {
  if (runtime.safety_source(kSafetySource) == mode) {
    return false;
  }
  runtime.set_safety_source(kSafetySource, mode);
  return true;
}

ArbiterDecision RecoveryArbiter::apply(const PluginRegistry& registry,
                                       const RecoveryAdvice& advice,
                                       TaskRuntime& runtime,
                                       std::int64_t now_ns) {
  const auto* d = registry.find(advice.plugin_id);
  if (d == nullptr || d->kind != PluginKind::kRecovery ||
      d->authority != PluginAuthority::kAdvice) {
    return ArbiterDecision::kRejected;
  }
  if (last_stamp_ns_ && advice.stamp_ns < *last_stamp_ns_) {
    return ArbiterDecision::kRejected;
  }

  const bool advice_stale = d->health_interval_ns > 0 &&
                            now_ns - advice.stamp_ns > 2 * d->health_interval_ns;
  if (advice_stale ||
      registry.liveness(advice.plugin_id, now_ns) != PluginLiveness::kLive) {
    last_reason_ = "recovery_plugin_stale";
    set_mode(runtime, SafetyMode::kHold);
    return ArbiterDecision::kStaleFailSafe;
  }
  last_stamp_ns_ = advice.stamp_ns;

  using Authority = lnr::RecoveryAuthority;
  if (advice.authority == Authority::Full) {
    if (!holding(runtime)) {
      return ArbiterDecision::kNoChange;
    }
    last_reason_.clear();
    set_mode(runtime, SafetyMode::kNormal);
    // resume() itself refuses while any other source still holds.
    const auto& task = runtime.active_task();
    if (advice.action == lnr::RecoveryAction::ResumeNavigation && task &&
        task->status == TaskStatus::kHolding) {
      runtime.resume();
    }
    return ArbiterDecision::kApplied;
  }

  if (advice.reason != lnr::RecoveryReason::None) {
    last_reason_ = std::string(lnr::RecoveryPlugin::name(advice.reason));
  } else if (last_reason_.empty() || last_reason_ == "recovery_plugin_stale") {
    last_reason_ = std::string(lnr::RecoveryPlugin::name(advice.authority));
  }

  bool changed = false;
  switch (advice.authority) {
    case Authority::Restricted:
      changed = set_mode(runtime, SafetyMode::kDegraded);
      break;
    case Authority::Suspended:
      changed = set_mode(runtime, SafetyMode::kHold);
      break;
    case Authority::Revoked:
      changed = set_mode(runtime, SafetyMode::kHold);
      changed = runtime.fail(last_reason_) || changed;
      break;
    case Authority::Full:
      break;
  }
  return changed ? ArbiterDecision::kApplied : ArbiterDecision::kNoChange;
}

}  // namespace motionslam::runtime
