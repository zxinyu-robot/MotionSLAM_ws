// End-to-end fault drills: FSR fault injection -> contract checks + LNR
// recovery plugin -> BT-side ContractGate / RecoveryArbiter -> TaskRuntime.
#include "contract/fault_injection.h"
#include "motionslam_runtime/contract_bridge.hpp"
#include "motionslam_runtime/recovery_bridge.hpp"

#include <cassert>

using namespace motionslam::runtime;
namespace lnr = legged_navigation;
using fsc::FaultKind;

namespace {

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kTick = 100 * kMs;

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
  t.world_reference = {"map.f1", 3, 2, "world"};
  t.skill_sequence = {{"goto", "elevator_hall"}, {"ride_elevator", "f2"}};
  return t;
}

// healthy_world() uses ns-scale validity windows; drills run on 100 ms
// ticks, so only the injected faults may expire anything.
fsc::FaultWorld drill_world(bool pose_anchored = false) {
  auto w = fsc::healthy_world();
  w.header.stamp.valid_until_ns = 0;
  w.lease.expiry_ns = 0;
  w.expected.pose_anchored = pose_anchored;
  w.header.asset = w.expected;
  w.lease.state = w.expected;
  return w;
}

struct Rig {
  PluginRegistry reg;
  RecoveryPluginHost host{reg};
  TaskRuntime rt;
  RecoveryArbiter arb;
  ContractGate gate;
  fsc::FaultWorld world;
  GateDecision gate_decision{GateDecision::kAccept};
  ArbiterDecision arb_decision{ArbiterDecision::kNoChange};

  explicit Rig(fsc::FaultWorld w = drill_world()) : world(std::move(w)) {
    assert(reg.add(health_source()) == RegisterResult::kRegistered);
    assert(host.attach() == RegisterResult::kRegistered);
    assert(rt.submit(intent("t1")) == SubmitResult::kAccepted);
    assert(rt.start());
  }

  // One control tick: inject, then the BT hub applies the contract gate and
  // the recovery advice for the same instant.
  void step(FaultKind kind, std::int64_t advance_ns = kTick, double amount = 0.0) {
    const auto r = fsc::inject(world, {kind, advance_ns, amount});
    gate_decision = gate.apply(r.admit, r.lease, rt);
    const auto advice = host.update(
        to_recovery_input(world.header.health, r.now_ns), r.now_ns);
    arb_decision = arb.apply(reg, advice, rt, r.now_ns);
  }
  void ticks(int n) {
    for (int i = 0; i < n; ++i) step(FaultKind::Noop);
  }
  TaskStatus status() const { return rt.active_task()->status; }
};

void test_mapping() {
  fsc::LocalizationHealth h;
  h.mode = fsc::LocalizationMode::Degraded2D;
  h.quality = 0.4;
  h.pose_jump_m = 0.2;
  h.alignment_fitness = 0.1;
  h.healthy_frames = 2;
  h.stance_ok = false;
  h.candidate_found = true;
  auto in = to_recovery_input(h, 1'234 * kMs);
  assert(in.timestamp_ms == 1'234);
  assert(in.localization_mode == 1);
  assert(in.tracking_valid);
  assert(in.quality == 0.4 && in.pose_jump_m == 0.2 && in.alignment_fitness == 0.1);
  assert(in.healthy_frames == 2 && !in.stance_ok);
  assert(in.descriptor_candidate_found && !in.alignment_complete);
  for (auto m : {fsc::LocalizationMode::Relocalizing, fsc::LocalizationMode::Hold}) {
    h.mode = m;
    assert(!to_recovery_input(h, 0).tracking_valid);
  }
  h.mode = fsc::LocalizationMode::Verifying;
  assert(to_recovery_input(h, 0).tracking_valid);
}

// source reports Verifying after alignment: LNR counts the verification
// frames instead of falling back to Searching; the contract keeps holding
// until Normal3D, then the BT resumes.
void test_verifying_counts_without_failing() {
  Rig r;
  r.step(FaultKind::LocalizationLost);
  r.ticks(6);
  auto& h = r.world.header.health;
  h.candidate_found = h.alignment_complete = h.alignment_success = true;
  h.mode = fsc::LocalizationMode::Verifying;
  h.reason = fsc::ReasonCode::None;
  r.step(FaultKind::Noop);  // Searching -> Aligning
  r.step(FaultKind::Noop);  // Aligning -> Verifying
  h.candidate_found = h.alignment_complete = h.alignment_success = false;
  r.ticks(3);  // healthy Verifying frames, no new candidate events
  assert(r.rt.safety_source(RecoveryArbiter::kSafetySource) == SafetyMode::kNormal);
  assert(r.rt.safety_source(ContractGate::kSafetySource) == SafetyMode::kHold);
  assert(r.status() == TaskStatus::kHolding);
  r.ticks(30);  // well past search_timeout_ms: no relocalization timeout
  assert(r.status() == TaskStatus::kHolding);
  r.step(FaultKind::Recover);
  assert(r.gate_decision == GateDecision::kAccept);
  assert(r.rt.resume());
}

// lost: contract holds at once; LNR confirms the loss and searches; after
// relocalization the gate clears first and the arbiter resumes the task.
void test_loss_hold_then_resume() {
  Rig r;
  r.step(FaultKind::Noop);
  assert(r.status() == TaskStatus::kRunning);

  r.step(FaultKind::LocalizationLost);
  assert(r.gate_decision == GateDecision::kHold);
  assert(r.gate.last_reason() == "localization_lost");
  assert(r.status() == TaskStatus::kHolding);
  assert(r.rt.safety_source(RecoveryArbiter::kSafetySource) == SafetyMode::kNormal);

  r.ticks(6);  // > loss_confirmation_ms
  assert(r.rt.safety_source(RecoveryArbiter::kSafetySource) == SafetyMode::kHold);
  assert(r.rt.safety_source(ContractGate::kSafetySource) == SafetyMode::kHold);

  r.step(FaultKind::Recover);  // Searching -> Aligning
  assert(r.gate_decision == GateDecision::kAccept);
  assert(r.rt.safety_source(ContractGate::kSafetySource) == SafetyMode::kNormal);
  assert(r.status() == TaskStatus::kHolding);  // recovery still holds

  r.ticks(4);  // Aligning -> Verifying -> 3 healthy frames
  assert(r.rt.safety_mode() == SafetyMode::kNormal);
  assert(r.status() == TaskStatus::kRunning);
}

// lost too long: LNR times out into SafeStop -> Revoked -> task failed.
void test_loss_timeout_fails() {
  Rig r;
  r.step(FaultKind::LocalizationLost);
  r.ticks(6 + 26);  // confirmation + search_timeout_ms
  assert(r.status() == TaskStatus::kFailed);
  assert(r.rt.active_task()->failure_reason == "relocalization_timeout");
  assert(r.rt.safety_mode() == SafetyMode::kHold);
}

// a short dropout is held by the contract only; the BT resumes explicitly.
void test_short_dropout_needs_explicit_resume() {
  Rig r;
  r.step(FaultKind::LocalizationLost);
  r.step(FaultKind::Recover);
  assert(r.gate_decision == GateDecision::kAccept);
  assert(r.rt.safety_mode() == SafetyMode::kNormal);
  assert(r.status() == TaskStatus::kHolding);
  assert(r.rt.resume());
}

// pose jump: Relocalizing -> contract Reverify; LNR names the jump.
void test_pose_jump() {
  Rig r;
  r.step(FaultKind::PoseJump, kTick, 2.0);
  assert(r.gate_decision == GateDecision::kHold);
  assert(r.gate.last_reason() == "localization_jump");
  r.ticks(6);
  assert(r.arb.last_reason() == "localization_jump");
  assert(r.status() == TaskStatus::kHolding);
}

// map / floor switch invalidates everything: Reject -> task failed.
void test_generation_bump_fails() {
  Rig r;
  r.step(FaultKind::GenerationBump);
  assert(r.gate_decision == GateDecision::kFailed);
  assert(r.status() == TaskStatus::kFailed);
  assert(r.rt.active_task()->failure_reason == "asset_stale");
  assert(r.arb_decision == ArbiterDecision::kNoChange);  // localization fine
}

// localization reset: pose-anchored state must be reverified; the lease
// issued on the old epoch holds the task until it is reissued.
void test_epoch_bump_reverify() {
  Rig r(drill_world(true));
  r.step(FaultKind::LocalizationEpochBump);
  assert(r.gate_decision == GateDecision::kHold);
  assert(r.status() == TaskStatus::kHolding);
  r.step(FaultKind::Recover);
  assert(r.gate_decision == GateDecision::kAccept);
  assert(r.rt.resume());

  Rig unanchored;
  unanchored.step(FaultKind::LocalizationEpochBump);
  assert(unanchored.gate_decision == GateDecision::kAccept);
  assert(unanchored.status() == TaskStatus::kRunning);
}

// link loss is report-only: the task keeps running under local autonomy.
void test_link_lost_keeps_running() {
  Rig r;
  r.step(FaultKind::LinkLost);
  r.ticks(10);
  assert(r.gate_decision == GateDecision::kAccept);
  assert(r.status() == TaskStatus::kRunning);
  assert(r.rt.safety_mode() == SafetyMode::kNormal);
}

void test_lease_faults_fail() {
  const std::pair<FaultKind, const char*> cases[] = {
      {FaultKind::LeaseExpired, "lease_expired"},
      {FaultKind::DeadlineMiss, "deadline_missed"},
      {FaultKind::WrongOwnerEpoch, "lease_stale_epoch"},
  };
  for (const auto& [kind, reason] : cases) {
    Rig r;
    r.step(kind);
    assert(r.gate_decision == GateDecision::kFailed);
    assert(r.status() == TaskStatus::kFailed);
    assert(r.rt.active_task()->failure_reason == reason);
  }
}

// gate Accept never clears an e-stop owned by another source.
void test_foreign_hold_survives_gate() {
  Rig r;
  r.step(FaultKind::LocalizationLost);
  r.rt.set_safety_source("estop", SafetyMode::kHold);
  r.step(FaultKind::Recover);
  assert(r.gate_decision == GateDecision::kAccept);
  assert(r.rt.safety_mode() == SafetyMode::kHold);
  assert(!r.rt.resume());
  r.rt.set_safety_source("estop", SafetyMode::kNormal);
  assert(r.rt.resume());
}

}  // namespace

int main() {
  test_mapping();
  test_loss_hold_then_resume();
  test_loss_timeout_fails();
  test_verifying_counts_without_failing();
  test_short_dropout_needs_explicit_resume();
  test_pose_jump();
  test_generation_bump_fails();
  test_epoch_bump_reverify();
  test_link_lost_keeps_running();
  test_lease_faults_fail();
  test_foreign_hold_survives_gate();
  return 0;
}
