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

## Build and test

```bash
cmake -S runtime/task_runtime -B build/runtime-task
cmake --build build/runtime-task
ctest --test-dir build/runtime-task --output-on-failure
```