#ifndef MOTIONSLAM_WATCHDOG_POLICY_HPP_
#define MOTIONSLAM_WATCHDOG_POLICY_HPP_

#include <cstdint>

namespace motionslam {

/// CmdVelForwarder: 超时无 /cmd_vel 输入是否应发零速
inline bool ShouldWatchdogZero(int64_t elapsed_ms, int64_t timeout_ms,
                               bool had_cmd, bool already_stopped) {
  if (!had_cmd) {
    return false;
  }
  return elapsed_ms > timeout_ms && !already_stopped;
}

/// PosCmdToTwist: 输入超时是否应发零速
inline bool ShouldZeroOnInputTimeout(int64_t cmd_age_ms, int64_t odom_age_ms,
                                     int64_t timeout_ms) {
  return cmd_age_ms > timeout_ms || odom_age_ms > timeout_ms;
}

}  // namespace motionslam

#endif  // MOTIONSLAM_WATCHDOG_POLICY_HPP_
