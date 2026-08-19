// TEST: VelocityLimiter 单元测试 (TDD 用例 5.1-①, 无 ROS 依赖可独立跑)
#include <gtest/gtest.h>

#include "motionslam/VelocityLimiter.hpp"
#include "motionslam/WatchdogPolicy.hpp"

namespace motionslam {

TEST(VelocityLimiter, ClampsForwardSpeed) {
  VelocityLimiter limiter({/*vx_min=*/-0.4, /*vx_max=*/1.0,
                           /*vy_abs=*/0.4, /*vyaw_abs=*/1.0});
  double vx = 2.0, vy = 0.0, vyaw = 0.0;
  limiter.Clamp(vx, vy, vyaw);
  EXPECT_DOUBLE_EQ(vx, 1.0);
}

TEST(VelocityLimiter, ClampsBackwardAndLateral) {
  VelocityLimiter limiter({-0.4, 1.0, 0.4, 1.0});
  double vx = -3.0, vy = -0.9, vyaw = 5.0;
  limiter.Clamp(vx, vy, vyaw);
  EXPECT_DOUBLE_EQ(vx, -0.4);
  EXPECT_DOUBLE_EQ(vy, -0.4);
  EXPECT_DOUBLE_EQ(vyaw, 1.0);
}

TEST(VelocityLimiter, PassesThroughInRange) {
  VelocityLimiter limiter({-0.4, 1.0, 0.4, 1.0});
  double vx = 0.5, vy = 0.2, vyaw = -0.3;
  limiter.Clamp(vx, vy, vyaw);
  EXPECT_DOUBLE_EQ(vx, 0.5);
  EXPECT_DOUBLE_EQ(vy, 0.2);
  EXPECT_DOUBLE_EQ(vyaw, -0.3);
}

TEST(WatchdogPolicy, ForwarderTimeoutMatchesPipelineYaml) {
  EXPECT_TRUE(ShouldWatchdogZero(401, 400, true, false));
  EXPECT_FALSE(ShouldWatchdogZero(399, 400, true, false));
}

}  // namespace motionslam
