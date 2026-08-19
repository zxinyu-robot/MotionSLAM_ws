// TEST: TrajTracker 单元测试 (纯逻辑, 无 ROS 依赖)
#include <gtest/gtest.h>

#include <cmath>

#include "motionslam/TrajTracker.hpp"

namespace motionslam {

namespace {
TrackerParams DefaultParams() {
  TrackerParams p;
  p.kp_pos = 1.0;
  p.kp_yaw = 1.5;
  p.max_pos_err = 1.0;
  p.deviation_thresh = 0.5;
  return p;
}
}  // namespace

// 无误差 + yaw=0: 期望速度直通机体系
TEST(TrajTracker, FeedforwardPassThroughAtZeroYaw) {
  TrajTracker tracker(DefaultParams());
  auto cmd = tracker.Compute(/*des_x=*/1.0, /*des_y=*/2.0,
                             /*des_vx=*/0.5, /*des_vy=*/0.2,
                             /*des_yaw=*/0.0, /*des_yaw_dot=*/0.0,
                             /*cur_x=*/1.0, /*cur_y=*/2.0, /*cur_yaw=*/0.0);
  EXPECT_NEAR(cmd.vx, 0.5, 1e-9);
  EXPECT_NEAR(cmd.vy, 0.2, 1e-9);
  EXPECT_NEAR(cmd.vyaw, 0.0, 1e-9);
  EXPECT_FALSE(cmd.deviated);
}

// 机体 yaw=90°: world 系 +x 速度应映射为机体 -y (右移)
TEST(TrajTracker, RotatesVelocityToBodyFrame) {
  TrajTracker tracker(DefaultParams());
  const double yaw90 = M_PI / 2.0;
  auto cmd = tracker.Compute(0.0, 0.0, /*des_vx=*/1.0, /*des_vy=*/0.0,
                             yaw90, 0.0, 0.0, 0.0, /*cur_yaw=*/yaw90);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-9);
  EXPECT_NEAR(cmd.vy, -1.0, 1e-9);
}

// 位置误差 P 反馈: 目标在正前方 0.3m, 静止期望速度 -> vx = kp*0.3
TEST(TrajTracker, PositionErrorFeedback) {
  TrajTracker tracker(DefaultParams());
  auto cmd = tracker.Compute(/*des_x=*/0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
                             /*cur_x=*/0.0, 0.0, 0.0);
  EXPECT_NEAR(cmd.vx, 0.3, 1e-9);
  EXPECT_FALSE(cmd.deviated);
}

// 大误差: 触发偏离标志, 且误差被 max_pos_err 饱和
TEST(TrajTracker, DeviationFlagAndErrorSaturation) {
  TrajTracker tracker(DefaultParams());
  auto cmd = tracker.Compute(/*des_x=*/5.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0);
  EXPECT_TRUE(cmd.deviated);
  EXPECT_NEAR(cmd.vx, 1.0, 1e-9);  // kp_pos * max_pos_err
}

// yaw 误差跨 ±pi 边界: 应走短弧
TEST(TrajTracker, YawErrorWrapsAroundPi) {
  TrajTracker tracker(DefaultParams());
  // 期望 yaw = 3.1, 当前 yaw = -3.1 -> 误差应为 -0.083 (短弧), 不是 +6.2
  auto cmd = tracker.Compute(0.0, 0.0, 0.0, 0.0,
                             /*des_yaw=*/3.1, 0.0, 0.0, 0.0,
                             /*cur_yaw=*/-3.1);
  EXPECT_LT(cmd.vyaw, 0.0);
  EXPECT_NEAR(cmd.vyaw, 1.5 * (3.1 - (-3.1) - 2.0 * M_PI), 1e-6);
}

// yaw_dot 前馈叠加
TEST(TrajTracker, YawDotFeedforward) {
  TrajTracker tracker(DefaultParams());
  auto cmd = tracker.Compute(0.0, 0.0, 0.0, 0.0,
                             /*des_yaw=*/0.0, /*des_yaw_dot=*/0.4,
                             0.0, 0.0, /*cur_yaw=*/0.0);
  EXPECT_NEAR(cmd.vyaw, 0.4, 1e-9);
}

}  // namespace motionslam
