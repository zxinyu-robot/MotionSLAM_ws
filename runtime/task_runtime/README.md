# Linux Task Runtime Core

This is an initial Linux-native C++20 lifecycle core. It has no ROS 2 dependency and is deliberately not connected to the existing ROS 2 overlay or planner yet.

## Responsibilities

- Accept a basic task intent with a stable task ID, world reference, and declared skill sequence.
- Enforce one non-terminal task at a time and reject task ID reuse within the process lifetime.
- Track task and current-skill lifecycle events.
- Pause, explicitly resume, cancel, or fail the active task.
- Fail closed in `HOLD` and unvalidated `DEGRADED` safety modes. Clearing the safety mode does not automatically resume a held task.

The runtime does not parse JSON, evaluate map-revision deltas, validate robot-specific capabilities, schedule priorities/deadlines, perform preemption, execute BT nodes, or command actuators. Those belong to the future Thin Harness, asset validation layer, BT adapter, and independent safety/controller modules.

## Lifecycle

```text
ACCEPTED -> RUNNING -> SUCCEEDED
               |  \-> FAILED
               |  \-> CANCELED
               |  \-> PAUSED -> RUNNING (explicit resume)
               \----> HOLDING -> RUNNING (safety normal + explicit resume)
```

The core stores execution policy fields but does not yet enforce deadlines, priority, or preemption. Task IDs are process-local and are not durable across restart.

## Plugin registry

`plugin_registry.hpp` defines the registration contract for runtime plugins (FSR components ①–⑥, LNR recovery, sources, transport). A plugin declares:

- `plugin_id` / `name` / `interface_version` — identity.
- `kind` — source, contract, token, memory, policy, recovery, transport, or custom.
- `authority` — `kEvidence` (observational data) or `kAdvice` (a suggested decision). Plugins can never hold execution authority; their output to the BT hub is only ever a candidate that the BT and safety layer may accept or reject.
- `provides` / `needs` — capability strings; `needs` must already be provided by registered plugins or registration fails with `kUnresolvedRequires`.
- `health_interval_ns` — if non-zero the plugin must report health within 2x the interval to stay `kLive`; no interval means always live once registered.

Registration failures: `kInvalidDescriptor`, `kDuplicateId`, `kUnresolvedRequires`. Removing a plugin recomputes the provided-capability set, so dependents can no longer register against a removed provider.

## LNR recovery bridge (optional)

`recovery_bridge.hpp` connects the LNR `RecoveryPlugin` ([legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)) to the registry and the task runtime. Built only with `-DMOTIONSLAM_WITH_LNR=ON`; LNR is fetched at a pinned commit (set `FETCHCONTENT_SOURCE_DIR_LEGGED_NAVIGATION_RUNTIME` to use a local checkout).

- `RecoveryPluginHost` registers the plugin as `kind=kRecovery`, `authority=kAdvice`, `provides={execution_authority_advice, recovery_action_advice}`, `needs={localization_health}`, and reports health on every update.
- `TaskRuntime::set_safety_source(source, mode)` keeps one mode per safety source (e-stop, recovery, ...); the effective mode is the most restrictive. `set_safety_mode()` is the `default` source.
- `RecoveryArbiter` is the BT-hub side and keeps execution authority. It only drives the `recovery` source, so it can never clear another source's hold:

| Advice | Runtime effect |
| --- | --- |
| `Full` | back to `NORMAL`; resume the held task only on `ResumeNavigation` |
| `Restricted` (Degraded2D) | `DEGRADED` (fails closed until degraded mode is validated) |
| `Suspended` (Searching/Aligning/Verifying) | `HOLD` |
| `Revoked` (SafeStop) | `HOLD` + fail the task with the reason code |
| plugin stale | `HOLD`, reason `recovery_plugin_stale` |

Advice from an unregistered plugin or an Evidence-level plugin is rejected, as is advice older than the last accepted one; advice older than 2x the health interval is treated as stale.

## FSR contract bridge and fault drills (optional)

`contract_bridge.hpp` connects the FSR contract ([fork-spatial-runtime](https://github.com/zxinyu-robot/fork-spatial-runtime) `contract/spatial_state.h`) to the runtime. Built only with `-DMOTIONSLAM_WITH_FSR=ON` (requires `MOTIONSLAM_WITH_LNR=ON`); FSR is fetched at a pinned commit (set `FETCHCONTENT_SOURCE_DIR_FORK_SPATIAL_RUNTIME` to use a local checkout).

- `to_recovery_input(LocalizationHealth, now_ns)` feeds the contract health, including its relocalization progress fields (`candidate_found` / `alignment_complete` / `alignment_success`), into the LNR plugin. Tracking is valid in `Normal3D` / `Degraded2D` / `Verifying` (LNR counts verification frames while the `ContractGate` still holds), invalid in `Relocalizing` / `Hold`.
- `ContractGate` is the BT-hub side of `admit()` and `dispose(lease)`. It drives only the `contract` safety source and never resumes a task:

| Contract result | Runtime effect |
| --- | --- |
| admit `Accept` + lease `Accept` | `contract` source back to `NORMAL` |
| admit `Reverify` / `Hold`, lease `StateMismatch` | `HOLD`, task kept |
| admit `Reject` | `HOLD` + fail with `name(reason)` (e.g. `asset_stale`) |
| lease `Expired` / `DeadlineMissed` / `StaleEpoch` / `WrongRobot` / `ClockMismatch` | `HOLD` + fail (`lease_expired`, `deadline_missed`, `lease_stale_epoch`, ...) |

`tests/fault_drill_tests.cpp` drives FSR `inject()` steps through `ContractGate` and the LNR `RecoveryPluginHost` + `RecoveryArbiter` into `TaskRuntime` on 100 ms ticks. Covered faults: localization lost (immediate contract hold → LNR search → relocalize → resume, plus a timeout case that ends in `RelocalizationTimeout`), short dropout, pose jump, generation bump, localization-epoch bump with and without pose-anchored state, link loss (task keeps running), lease expired / deadline missed / stale owner epoch, and an e-stop that outlives the gate.

## Build and test

```bash
cmake -S runtime/task_runtime -B build/runtime-task
cmake --build build/runtime-task
ctest --test-dir build/runtime-task --output-on-failure

# with the LNR recovery bridge
cmake -S runtime/task_runtime -B build/runtime-task -DMOTIONSLAM_WITH_LNR=ON

# with the FSR contract bridge and fault drills
cmake -S runtime/task_runtime -B build/runtime-task -DMOTIONSLAM_WITH_LNR=ON -DMOTIONSLAM_WITH_FSR=ON
```
