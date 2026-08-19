#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "motionslam_ipc/shm_cloud_ring.hpp"

namespace motionslam {

struct CloudObstacleContext {
  uint64_t generation{0};
  uint64_t monotonic_ns{0};
  uint64_t session_token{0};
  uint64_t tile_token{0};
  bool ready{false};
};

struct CloudObstacleFilter {
  double min_z{0.10};
  double max_z{1.50};
  double min_range{0.15};
  double max_range{3.0};
  uint64_t cloud_max_age_ns{500000000ULL};
  uint64_t context_max_age_ns{1000000000ULL};
};

struct CloudObstacleGrid {
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

struct CloudObstacleDecision {
  bool accepted{false};
  bool new_frame{false};
  std::string reason;
};

class ShmCloudObstacleLayerCore {
 public:
  static uint64_t SystemNowNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
  }

  static uint64_t MonotonicNowNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

  CloudObstacleDecision Accept(
      const motionslam_ipc::CloudFrame& cloud,
      const CloudObstacleContext& before,
      const CloudObstacleContext& after,
      const CloudObstacleFilter& filter,
      uint64_t system_now_ns = 0,
      uint64_t monotonic_now_ns = 0) {
    CloudObstacleDecision result;
    const uint64_t system_now =
        system_now_ns == 0 ? SystemNowNs() : system_now_ns;
    const uint64_t monotonic_now =
        monotonic_now_ns == 0 ? MonotonicNowNs() : monotonic_now_ns;

    if (!SameContextSnapshot(before, after)) {
      result.reason = "context_changed_during_cloud_snapshot";
      return result;
    }
    if (!after.ready || after.generation == 0 ||
        after.session_token == 0 || after.tile_token == 0) {
      result.reason = "context_not_ready";
      return result;
    }
    if (after.monotonic_ns == 0 || monotonic_now < after.monotonic_ns ||
        (filter.context_max_age_ns != 0 &&
         monotonic_now - after.monotonic_ns > filter.context_max_age_ns)) {
      result.reason = "context_stale";
      return result;
    }
    const uint64_t cloud_stamp =
        cloud.timestamp_ns > 0 ? static_cast<uint64_t>(cloud.timestamp_ns) : 0;
    if (cloud.sequence == 0 || cloud.generation == 0 || cloud_stamp == 0 ||
        system_now < cloud_stamp ||
        (filter.cloud_max_age_ns != 0 &&
         system_now - cloud_stamp > filter.cloud_max_age_ns)) {
      result.reason = "cloud_stale_or_invalid";
      return result;
    }
    if (cloud.session_id != after.session_token) {
      result.reason = "cloud_session_token_mismatch";
      return result;
    }
    if (cloud.tile_id != after.tile_token) {
      result.reason = "cloud_tile_token_mismatch";
      return result;
    }

    result.accepted = true;
    result.new_frame =
        cloud.sequence != sequence_ || cloud.generation != cloud_generation_ ||
        cloud.session_id != cloud_session_ || cloud.tile_id != cloud_tile_ ||
        after.generation != context_generation_;
    sequence_ = cloud.sequence;
    cloud_generation_ = cloud.generation;
    cloud_session_ = cloud.session_id;
    cloud_tile_ = cloud.tile_id;
    context_generation_ = after.generation;
    return result;
  }

  static std::vector<uint8_t> Rasterize(
      const std::vector<motionslam_ipc::PointXYZI>& points,
      const CloudObstacleGrid& grid,
      const CloudObstacleFilter& filter,
      double robot_x,
      double robot_y,
      uint8_t lethal_cost) {
    std::vector<uint8_t> cells(
        static_cast<size_t>(grid.size_x) * grid.size_y, 0U);
    if (grid.size_x == 0 || grid.size_y == 0 ||
        !std::isfinite(grid.resolution) || grid.resolution <= 0.0) {
      return cells;
    }
    const double min_range_sq = filter.min_range * filter.min_range;
    const double max_range_sq = filter.max_range * filter.max_range;
    for (const auto& point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || point.z < filter.min_z ||
          point.z > filter.max_z) {
        continue;
      }
      const double dx = static_cast<double>(point.x) - robot_x;
      const double dy = static_cast<double>(point.y) - robot_y;
      const double range_sq = dx * dx + dy * dy;
      if (range_sq < min_range_sq || range_sq > max_range_sq ||
          point.x < grid.origin_x || point.y < grid.origin_y) {
        continue;
      }
      const auto mx = static_cast<unsigned int>(
          (static_cast<double>(point.x) - grid.origin_x) / grid.resolution);
      const auto my = static_cast<unsigned int>(
          (static_cast<double>(point.y) - grid.origin_y) / grid.resolution);
      if (mx < grid.size_x && my < grid.size_y) {
        cells[static_cast<size_t>(my) * grid.size_x + mx] = lethal_cost;
      }
    }
    return cells;
  }

  void Reset() {
    sequence_ = 0;
    cloud_generation_ = 0;
    cloud_session_ = 0;
    cloud_tile_ = 0;
    context_generation_ = 0;
  }

 private:
  static bool SameContextSnapshot(const CloudObstacleContext& lhs,
                                  const CloudObstacleContext& rhs) {
    return lhs.generation == rhs.generation &&
           lhs.monotonic_ns == rhs.monotonic_ns &&
           lhs.session_token == rhs.session_token &&
           lhs.tile_token == rhs.tile_token && lhs.ready == rhs.ready;
  }

  uint64_t sequence_{0};
  uint64_t cloud_generation_{0};
  uint64_t cloud_session_{0};
  uint64_t cloud_tile_{0};
  uint64_t context_generation_{0};
};

}  // namespace motionslam
