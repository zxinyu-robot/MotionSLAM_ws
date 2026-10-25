#pragma once

#include <cstdint>
#include <string>

#include "contract/spatial_state.h"
#include "legged_navigation/recovery_plugin.h"
#include "motionslam_runtime/task_runtime.hpp"

namespace motionslam::runtime {

namespace fsc = fork_spatial::contract;

// Relocalization progress reported by the localization source (e.g. Super-LIO
// /lio/relocation/*). LocalizationHealth does not carry it, but the LNR state
// machine needs it to leave Searching/Aligning.
struct RelocalizationProgress {
  bool candidate_found{false};
  bool alignment_complete{false};
  bool alignment_success{false};
};

// FSR LocalizationHealth -> LNR RecoveryInput. Tracking is valid only while
// the contract says the localization is usable (Normal3D / Degraded2D).
legged_navigation::RecoveryInput to_recovery_input(
    const fsc::LocalizationHealth& health, std::int64_t now_ns,
    const RelocalizationProgress& progress = {});

// Failure reason for a non-Accept lease disposition; contract ReasonCode
// names where one exists, "lease_<disposition>" otherwise.
std::string lease_reason(fsc::LeaseDisposition lease);

enum class GateDecision {
  kAccept,  // state and lease admitted; "contract" source cleared
  kHold,    // Reverify / Hold / lease StateMismatch: task held, kept
  kFailed,  // Reject or a lease that can never become valid: task failed
};

// BT-hub side gate for the FSR contract checks admit() and dispose(lease).
// Drives only the "contract" safety source; clearing it never resumes a
// held task (the BT resumes explicitly once every source is normal).
//   admit Accept + lease Accept -> NORMAL
//   admit Reverify / Hold       -> HOLD
//   lease StateMismatch         -> HOLD (lease must be reissued on new state)
//   admit Reject                -> HOLD + fail(name(reason))
//   lease Expired / DeadlineMissed / StaleEpoch / WrongRobot / ClockMismatch
//                               -> HOLD + fail(lease_reason)
class ContractGate final {
 public:
  GateDecision apply(const fsc::AdmitResult& admit, fsc::LeaseDisposition lease,
                     TaskRuntime& runtime);

  static constexpr const char* kSafetySource = "contract";

  const std::string& last_reason() const noexcept { return last_reason_; }

 private:
  std::string last_reason_;
};

}  // namespace motionslam::runtime
