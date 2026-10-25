#include "motionslam_runtime/contract_bridge.hpp"

namespace motionslam::runtime {

namespace lnr = legged_navigation;

// RecoveryInput::localization_mode is documented as the contract enum value.
static_assert(static_cast<std::uint8_t>(fsc::LocalizationMode::Normal3D) == 0);
static_assert(static_cast<std::uint8_t>(fsc::LocalizationMode::Degraded2D) == 1);
static_assert(static_cast<std::uint8_t>(fsc::LocalizationMode::Relocalizing) == 2);
static_assert(static_cast<std::uint8_t>(fsc::LocalizationMode::Verifying) == 3);
static_assert(static_cast<std::uint8_t>(fsc::LocalizationMode::Hold) == 4);

lnr::RecoveryInput to_recovery_input(const fsc::LocalizationHealth& health,
                                     std::int64_t now_ns,
                                     const RelocalizationProgress& progress) {
  lnr::RecoveryInput in;
  in.timestamp_ms = now_ns / 1'000'000;
  in.localization_mode = static_cast<std::uint8_t>(health.mode);
  in.quality = health.quality;
  in.pose_jump_m = health.pose_jump_m;
  in.alignment_fitness = health.alignment_fitness;
  in.healthy_frames = health.healthy_frames;
  in.stance_ok = health.stance_ok;
  in.tracking_valid = health.usable();
  in.descriptor_candidate_found = progress.candidate_found;
  in.alignment_complete = progress.alignment_complete;
  in.alignment_success = progress.alignment_success;
  return in;
}

std::string lease_reason(fsc::LeaseDisposition lease) {
  using L = fsc::LeaseDisposition;
  switch (lease) {
    case L::Accept: return {};
    case L::Expired: return std::string(fsc::name(fsc::ReasonCode::LeaseExpired));
    case L::DeadlineMissed:
      return std::string(fsc::name(fsc::ReasonCode::DeadlineMissed));
    case L::StateMismatch: return std::string(fsc::name(fsc::ReasonCode::AssetStale));
    case L::StaleEpoch: return "lease_stale_epoch";
    case L::WrongRobot: return "lease_wrong_robot";
    case L::ClockMismatch: return "lease_clock_mismatch";
  }
  return "lease_unknown";
}

GateDecision ContractGate::apply(const fsc::AdmitResult& admit,
                                 fsc::LeaseDisposition lease,
                                 TaskRuntime& runtime) {
  using A = fsc::AdmitDisposition;
  using L = fsc::LeaseDisposition;

  std::string fail_reason;
  if (admit.disposition == A::Reject) {
    fail_reason = std::string(fsc::name(admit.reason));
  } else if (lease != L::Accept && lease != L::StateMismatch) {
    fail_reason = lease_reason(lease);
  }
  if (!fail_reason.empty()) {
    last_reason_ = fail_reason;
    runtime.set_safety_source(kSafetySource, SafetyMode::kHold);
    runtime.fail(last_reason_);
    return GateDecision::kFailed;
  }

  if (admit.disposition != A::Accept || lease == L::StateMismatch) {
    last_reason_ = admit.disposition != A::Accept
                       ? std::string(fsc::name(admit.reason))
                       : lease_reason(lease);
    runtime.set_safety_source(kSafetySource, SafetyMode::kHold);
    return GateDecision::kHold;
  }

  last_reason_.clear();
  runtime.set_safety_source(kSafetySource, SafetyMode::kNormal);
  return GateDecision::kAccept;
}

}  // namespace motionslam::runtime
