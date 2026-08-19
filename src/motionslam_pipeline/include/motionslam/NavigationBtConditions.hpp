#pragma once

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "motionslam/MapSessionManager.hpp"

namespace motionslam {

struct ContextCheckResult {
  bool ok{false};
  std::string reason;
  NavigationContextState state{};
};

struct GoalCheckResult {
  bool ok{false};
  std::string reason;
};

// Read-only, seqlock-consistent validation used by the BT conditions and tests.
ContextCheckResult CheckNavigationContext(
    const std::string& state_page_path, uint64_t max_age_ns,
    uint64_t now_monotonic_ns = 0);

// Re-reads the context around the map access and rejects a context switch.
GoalCheckResult CheckGoalAgainstSharedMap(
    const geometry_msgs::msg::PoseStamped& goal,
    const std::string& state_page_path, const std::string& map_shm_name,
    uint64_t expected_generation, const std::string& expected_session,
    uint64_t max_age_ns, int lethal_threshold = 100,
    uint64_t now_monotonic_ns = 0, bool strict_protocol = true);

}  // namespace motionslam
