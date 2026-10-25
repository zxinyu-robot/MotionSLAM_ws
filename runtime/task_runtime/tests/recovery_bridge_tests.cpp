#include "motionslam_runtime/recovery_bridge.hpp"

#include <cassert>

using namespace motionslam::runtime;
namespace lnr = legged_navigation;

namespace {

constexpr std::int64_t kMs = 1'000'000;

PluginDescriptor health_source() {
  PluginDescriptor d;
  d.plugin_id = "super-lio";
  d.name = "Super-LIO";
  d.interface_version = "v1";
  d.kind = PluginKind::kSource;
  d.authority = PluginAuthority::kEvidence;
  d.provides = {kCapLocalizationHealth};
  return d;
}

TaskIntent intent(const char* id) {
  TaskIntent t;
  t.task_id = id;
  t.task_type = "navigate";
  t.world_reference = {"map.f1", 3, 2, "map"};
  t.skill_sequence = {{"goto", "elevator_hall"}, {"ride_elevator", "f2"}};
  return t;
}

lnr::RecoveryInput frame(std::int64_t t_ms, bool tracking = true) {
  lnr::RecoveryInput in;
  in.timestamp_ms = t_ms;
  in.tracking_valid = tracking;
  return in;
}

void start_task(TaskRuntime& rt, const char* id) {
  assert(rt.submit(intent(id)) == SubmitResult::kAccepted);
  assert(rt.start());
}

void test_registration_needs_health_source() {
  PluginRegistry reg;
  RecoveryPluginHost host(reg);
  assert(host.attach() == RegisterResult::kUnresolvedRequires);
  assert(!host.attached());
  assert(reg.add(health_source()) == RegisterResult::kRegistered);
  assert(host.attach() == RegisterResult::kRegistered);
  const auto* d = reg.find("lnr-recovery");
  assert(d != nullptr);
  assert(d->kind == PluginKind::kRecovery);
  assert(d->authority == PluginAuthority::kAdvice);
  assert(reg.providers(kCapExecutionAuthorityAdvice).size() == 1);
  assert(host.detach());
  assert(reg.find("lnr-recovery") == nullptr);
}

// lost -> Hold, relocalized -> Normal + explicit resume by the arbiter.
void test_loss_holds_and_recovery_resumes() {
  PluginRegistry reg;
  reg.add(health_source());
  RecoveryPluginHost host(reg);
  assert(host.attach() == RegisterResult::kRegistered);
  TaskRuntime rt;
  RecoveryArbiter arb;
  start_task(rt, "t1");

  std::int64_t t = 0;
  auto step = [&](lnr::RecoveryInput in) {
    const auto now = t * kMs;
    return arb.apply(reg, host.update(in, now), rt, now);
  };
  assert(step(frame(t)) == ArbiterDecision::kNoChange);
  t = 100;
  step(frame(t, false));
  t = 700;  // past loss confirmation
  step(frame(t, false));
  assert(rt.active_task()->status == TaskStatus::kHolding);
  assert(rt.safety_mode() == SafetyMode::kHold);
  assert(arb.holding(rt));
  assert(arb.last_reason() == "LocalizationLost");

  // candidate -> alignment -> verify -> healthy frames
  auto in = frame(t += 50, false);
  in.descriptor_candidate_found = true;
  step(in);
  in = frame(t += 50, false);
  in.alignment_complete = true;
  in.alignment_success = true;
  in.alignment_fitness = 0.1;
  step(in);
  for (int i = 0; i < 6 && rt.active_task()->status != TaskStatus::kRunning;
       ++i) {
    step(frame(t += 50));
  }
  assert(rt.safety_mode() == SafetyMode::kNormal);
  assert(rt.active_task()->status == TaskStatus::kRunning);
  assert(!arb.holding(rt));
}

// relocalization timeout -> Revoked -> task failed with the reason code.
void test_timeout_revokes_and_fails_task() {
  PluginRegistry reg;
  reg.add(health_source());
  RecoveryPluginHost host(reg);
  host.attach();
  TaskRuntime rt;
  RecoveryArbiter arb;
  start_task(rt, "t2");
  for (std::int64_t t = 0; t <= 4000; t += 100) {
    arb.apply(reg, host.update(frame(t, false), t * kMs), rt, t * kMs);
  }
  assert(rt.active_task()->status == TaskStatus::kFailed);
  assert(rt.active_task()->failure_reason == "RelocalizationTimeout");
  assert(rt.safety_mode() == SafetyMode::kHold);
}

// Degraded2D -> Restricted -> Degraded mode (fails closed in TaskRuntime).
void test_degraded_mode_restricts() {
  PluginRegistry reg;
  reg.add(health_source());
  RecoveryPluginHost host(reg);
  host.attach();
  TaskRuntime rt;
  RecoveryArbiter arb;
  start_task(rt, "t3");
  auto in = frame(0);
  in.localization_mode = 1;
  assert(arb.apply(reg, host.update(in, 0), rt, 0) == ArbiterDecision::kApplied);
  assert(rt.safety_mode() == SafetyMode::kDegraded);
  assert(rt.active_task()->status == TaskStatus::kHolding);
  // back to Normal3D: mode clears; no ResumeNavigation, so task stays held
  assert(arb.apply(reg, host.update(frame(50), 50 * kMs), rt, 50 * kMs) ==
         ArbiterDecision::kApplied);
  assert(rt.safety_mode() == SafetyMode::kNormal);
  assert(rt.active_task()->status == TaskStatus::kHolding);
}

// Plugin stops reporting -> BT holds on its own authority.
void test_stale_plugin_fails_safe() {
  PluginRegistry reg;
  reg.add(health_source());
  RecoveryPluginHost host(reg, {}, 100 * kMs);
  host.attach();
  TaskRuntime rt;
  RecoveryArbiter arb;
  start_task(rt, "t4");
  const auto advice = host.update(frame(0), 0);
  assert(arb.apply(reg, advice, rt, 50 * kMs) == ArbiterDecision::kNoChange);
  assert(arb.apply(reg, advice, rt, 500 * kMs) ==
         ArbiterDecision::kStaleFailSafe);
  assert(rt.safety_mode() == SafetyMode::kHold);
  assert(arb.last_reason() == "recovery_plugin_stale");
  // fresh Full advice clears the hold the arbiter itself placed
  const auto fresh = host.update(frame(600), 600 * kMs);
  assert(arb.apply(reg, fresh, rt, 600 * kMs) == ArbiterDecision::kApplied);
  assert(rt.safety_mode() == SafetyMode::kNormal);
}

// Advice from unregistered or Evidence-level plugins is ignored.
void test_boundaries() {
  PluginRegistry reg;
  reg.add(health_source());
  RecoveryPluginHost host(reg);
  TaskRuntime rt;
  RecoveryArbiter arb;
  start_task(rt, "t5");
  auto advice = host.update(frame(0, false), 0);  // not attached yet
  assert(arb.apply(reg, advice, rt, 0) == ArbiterDecision::kRejected);
  advice.plugin_id = "super-lio";  // Evidence-level source
  advice.authority = lnr::RecoveryAuthority::Revoked;
  assert(arb.apply(reg, advice, rt, 0) == ArbiterDecision::kRejected);
  assert(rt.active_task()->status == TaskStatus::kRunning);
}

RecoveryAdvice advice_at(const RecoveryPluginHost& host,
                         lnr::RecoveryAuthority authority, std::int64_t ns,
                         lnr::RecoveryAction action = lnr::RecoveryAction::None) {
  (void)host;
  RecoveryAdvice a;
  a.plugin_id = "lnr-recovery";
  a.authority = authority;
  a.action = action;
  a.stamp_ns = ns;
  return a;
}

// An e-stop asserted after a recovery hold survives Full+ResumeNavigation.
void test_foreign_hold_survives_recovery_clear() {
  PluginRegistry reg;
  reg.add(health_source());
  RecoveryPluginHost host(reg);
  host.attach();
  TaskRuntime rt;
  RecoveryArbiter arb;
  start_task(rt, "t6");
  host.update(frame(0), 0);
  arb.apply(reg, advice_at(host, lnr::RecoveryAuthority::Suspended, 0), rt, 0);
  assert(rt.safety_mode() == SafetyMode::kHold);
  rt.set_safety_source("estop", SafetyMode::kHold);
  host.update(frame(50), 50 * kMs);
  assert(arb.apply(reg,
                   advice_at(host, lnr::RecoveryAuthority::Full, 50 * kMs,
                             lnr::RecoveryAction::ResumeNavigation),
                   rt, 50 * kMs) == ArbiterDecision::kApplied);
  assert(!arb.holding(rt));
  assert(rt.safety_mode() == SafetyMode::kHold);
  assert(rt.active_task()->status == TaskStatus::kHolding);
  rt.set_safety_source("estop", SafetyMode::kNormal);
  assert(rt.safety_mode() == SafetyMode::kNormal);
  assert(rt.active_task()->status == TaskStatus::kHolding);  // explicit resume
}

// Revoked fails the task even while another source owns HOLD or DEGRADED.
void test_revoked_under_foreign_mode() {
  for (const auto mode : {SafetyMode::kHold, SafetyMode::kDegraded}) {
    PluginRegistry reg;
    reg.add(health_source());
    RecoveryPluginHost host(reg);
    host.attach();
    TaskRuntime rt;
    RecoveryArbiter arb;
    start_task(rt, "t7");
    rt.set_safety_source("estop", mode);
    host.update(frame(0), 0);
    auto revoked = advice_at(host, lnr::RecoveryAuthority::Revoked, 0);
    revoked.reason = lnr::RecoveryReason::RelocalizationTimeout;
    assert(arb.apply(reg, revoked, rt, 0) == ArbiterDecision::kApplied);
    assert(rt.active_task()->status == TaskStatus::kFailed);
    assert(rt.active_task()->failure_reason == "RelocalizationTimeout");
    assert(rt.safety_source("estop") == mode);
  }
}

// Older advice cannot override a newer decision; old advice counts as stale.
void test_advice_ordering_and_freshness() {
  PluginRegistry reg;
  reg.add(health_source());
  RecoveryPluginHost host(reg, {}, 100 * kMs);
  host.attach();
  TaskRuntime rt;
  RecoveryArbiter arb;
  start_task(rt, "t8");
  host.update(frame(150), 150 * kMs);
  const auto old_full = advice_at(host, lnr::RecoveryAuthority::Full, 100 * kMs,
                                  lnr::RecoveryAction::ResumeNavigation);
  arb.apply(reg, advice_at(host, lnr::RecoveryAuthority::Suspended, 150 * kMs),
            rt, 150 * kMs);
  assert(rt.safety_mode() == SafetyMode::kHold);
  assert(arb.apply(reg, old_full, rt, 160 * kMs) == ArbiterDecision::kRejected);
  assert(rt.safety_mode() == SafetyMode::kHold);

  // heartbeat is fresh but the advice itself is older than 2x the interval
  host.update(frame(500), 500 * kMs);
  const auto late = advice_at(host, lnr::RecoveryAuthority::Full, 200 * kMs);
  assert(arb.apply(reg, late, rt, 500 * kMs) == ArbiterDecision::kStaleFailSafe);
  assert(rt.safety_mode() == SafetyMode::kHold);
}

}  // namespace

int main() {
  test_registration_needs_health_source();
  test_loss_holds_and_recovery_resumes();
  test_timeout_revokes_and_fails_task();
  test_degraded_mode_restricts();
  test_stale_plugin_fails_safe();
  test_boundaries();
  test_foreign_hold_survives_recovery_clear();
  test_revoked_under_foreign_mode();
  test_advice_ordering_and_freshness();
  return 0;
}
