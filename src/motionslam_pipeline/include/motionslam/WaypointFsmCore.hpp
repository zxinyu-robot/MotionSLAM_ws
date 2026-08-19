#ifndef MOTIONSLAM_WAYPOINT_FSM_CORE_HPP_
#define MOTIONSLAM_WAYPOINT_FSM_CORE_HPP_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace motionslam {

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
  double yaw_deg = 0.0;
};

struct WaypointFsmParams {
  double arrive_dist = 0.3;
  double arrive_yaw_deg = 15.0;
  double stable_time_s = 1.0;
  double goal_timeout_s = 120.0;
  double max_speed_for_stable = 0.15;
  int max_retry = 2;
  double min_battery_voltage = 22.0;
  bool enable_battery_check = false;
};

enum class WaypointFsmState {
  IDLE,
  WAIT_ODOM,
  NAVIGATING,
  WAIT_STABLE,
  FINISHED,
  ABORTED,
};

inline const char* WaypointFsmStateName(WaypointFsmState s) {
  switch (s) {
    case WaypointFsmState::IDLE:
      return "IDLE";
    case WaypointFsmState::WAIT_ODOM:
      return "WAIT_ODOM";
    case WaypointFsmState::NAVIGATING:
      return "NAVIGATING";
    case WaypointFsmState::WAIT_STABLE:
      return "WAIT_STABLE";
    case WaypointFsmState::FINISHED:
      return "FINISHED";
    case WaypointFsmState::ABORTED:
      return "ABORTED";
  }
  return "UNKNOWN";
}

struct OdomSample {
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
  double speed = 0.0;
};

/// 纯逻辑 FSM (无 ROS), 便于单测
class WaypointFsmCore {
 public:
  explicit WaypointFsmCore(WaypointFsmParams params = {}) : params_(params) {}

  void SetWaypoints(std::vector<Waypoint> wps) {
    waypoints_ = std::move(wps);
    ResetRun();
  }

  const std::vector<Waypoint>& waypoints() const { return waypoints_; }

  WaypointFsmState state() const { return state_; }
  size_t current_index() const { return index_; }
  int retry_count() const { return retry_; }

  void ResetRun() {
    state_ = waypoints_.empty() ? WaypointFsmState::FINISHED
                                : WaypointFsmState::WAIT_ODOM;
    index_ = 0;
    retry_ = 0;
    stable_accum_s_ = 0.0;
    nav_elapsed_s_ = 0.0;
    stop_requested_ = false;
    low_battery_ = false;
    last_error_.clear();
  }

  void RequestStop() { stop_requested_ = true; }

  void SetLowBattery(bool low) { low_battery_ = low; }

  const std::string& last_error() const { return last_error_; }

  /// @param dt_s 控制周期
  /// @return 若需发布新 goal, 返回目标航点
  std::optional<Waypoint> Tick(double dt_s, bool has_odom,
                               const OdomSample& odom) {
    if (stop_requested_) {
      state_ = WaypointFsmState::ABORTED;
      last_error_ = "stop requested";
      return std::nullopt;
    }
    if (params_.enable_battery_check && low_battery_) {
      state_ = WaypointFsmState::ABORTED;
      last_error_ = "low battery";
      return std::nullopt;
    }
    if (waypoints_.empty()) {
      state_ = WaypointFsmState::FINISHED;
      return std::nullopt;
    }

    switch (state_) {
      case WaypointFsmState::WAIT_ODOM:
        if (!has_odom) {
          return std::nullopt;
        }
        state_ = WaypointFsmState::NAVIGATING;
        nav_elapsed_s_ = 0.0;
        retry_ = 0;
        stable_accum_s_ = 0.0;
        return waypoints_[index_];

      case WaypointFsmState::NAVIGATING:
        if (!has_odom) {
          return std::nullopt;
        }
        nav_elapsed_s_ += dt_s;
        if (nav_elapsed_s_ > params_.goal_timeout_s) {
          if (retry_ < params_.max_retry) {
            ++retry_;
            nav_elapsed_s_ = 0.0;
            return waypoints_[index_];
          }
          if (index_ + 1 < waypoints_.size()) {
            last_error_ = "timeout skip waypoint";
            ++index_;
            retry_ = 0;
            nav_elapsed_s_ = 0.0;
            return waypoints_[index_];
          }
          state_ = WaypointFsmState::ABORTED;
          last_error_ = "timeout on last waypoint";
          return std::nullopt;
        }
        if (IsArrived(odom, waypoints_[index_])) {
          state_ = WaypointFsmState::WAIT_STABLE;
          stable_accum_s_ = 0.0;
        }
        return std::nullopt;

      case WaypointFsmState::WAIT_STABLE:
        if (!has_odom) {
          return std::nullopt;
        }
        if (odom.speed <= params_.max_speed_for_stable) {
          stable_accum_s_ += dt_s;
        } else {
          stable_accum_s_ = 0.0;
        }
        if (stable_accum_s_ >= params_.stable_time_s) {
          if (index_ + 1 >= waypoints_.size()) {
            state_ = WaypointFsmState::FINISHED;
            return std::nullopt;
          }
          ++index_;
          retry_ = 0;
          nav_elapsed_s_ = 0.0;
          state_ = WaypointFsmState::NAVIGATING;
          return waypoints_[index_];
        }
        return std::nullopt;

      case WaypointFsmState::FINISHED:
      case WaypointFsmState::ABORTED:
      case WaypointFsmState::IDLE:
      default:
        return std::nullopt;
    }
  }

  static double NormalizeAngleRad(double a) {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a <= -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  static double YawDegToRad(double deg) { return deg * M_PI / 180.0; }

  bool IsArrived(const OdomSample& odom, const Waypoint& wp) const {
    const double dx = wp.x - odom.x;
    const double dy = wp.y - odom.y;
    if (std::hypot(dx, dy) > params_.arrive_dist) {
      return false;
    }
    const double yaw_err = std::abs(
        NormalizeAngleRad(YawDegToRad(wp.yaw_deg) - odom.yaw_rad));
    return yaw_err <= YawDegToRad(params_.arrive_yaw_deg);
  }

 private:
  WaypointFsmParams params_;
  std::vector<Waypoint> waypoints_;
  WaypointFsmState state_ = WaypointFsmState::IDLE;
  size_t index_ = 0;
  int retry_ = 0;
  double stable_accum_s_ = 0.0;
  double nav_elapsed_s_ = 0.0;
  bool stop_requested_ = false;
  bool low_battery_ = false;
  std::string last_error_;
};

}  // namespace motionslam

#endif  // MOTIONSLAM_WAYPOINT_FSM_CORE_HPP_
