#ifndef MOTIONSLAM_TRAJ_TRACKER_HPP_
#define MOTIONSLAM_TRAJ_TRACKER_HPP_

#include <algorithm>
#include <cmath>

namespace motionslam {

/**
 * @brief 轨迹跟踪纯逻辑 (无 ROS 依赖, 便于单测)。
 *
 * 输入: ego-planner traj_server 的 PositionCommand (world 系期望位置/速度/yaw)
 *       + 当前 odom (world 系位置/yaw)。
 * 输出: Go2 机体系速度指令 (vx 前向, vy 左向, vyaw)。
 *
 * 控制律: 前馈期望速度 + 位置误差 P 反馈, 再旋转到机体系;
 *         yaw 通道为期望 yaw_dot 前馈 + yaw 误差 P 反馈。
 */
struct TrackerParams {
  double kp_pos = 1.0;         ///< 位置误差反馈增益 (1/s)
  double kp_yaw = 1.5;         ///< 偏航误差反馈增益 (1/s)
  double max_pos_err = 1.0;    ///< 位置误差饱和 (m), 防止大误差下速度爆冲
  double deviation_thresh = 0.5;  ///< 偏离轨迹阈值 (m), 超过应触发重规划
};

struct TrackerCmd {
  double vx = 0.0;
  double vy = 0.0;
  double vyaw = 0.0;
  bool deviated = false;  ///< 偏离轨迹超阈值 (调用方应触发 replan/停止)
};

class TrajTracker {
 public:
  explicit TrajTracker(const TrackerParams& params) : params_(params) {}

  /// 将角度归一化到 (-pi, pi]
  static double NormalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
  }

  /**
   * @brief 计算一拍机体系速度指令。
   * @param des_x,des_y     期望位置 (world)
   * @param des_vx,des_vy   期望速度 (world)
   * @param des_yaw         期望偏航 (world)
   * @param des_yaw_dot     期望偏航角速度
   * @param cur_x,cur_y     当前位置 (world)
   * @param cur_yaw         当前偏航 (world)
   */
  TrackerCmd Compute(double des_x, double des_y,
                     double des_vx, double des_vy,
                     double des_yaw, double des_yaw_dot,
                     double cur_x, double cur_y, double cur_yaw) const {
    TrackerCmd cmd;

    double ex = des_x - cur_x;
    double ey = des_y - cur_y;
    const double err_norm = std::hypot(ex, ey);
    cmd.deviated = err_norm > params_.deviation_thresh;
    if (err_norm > params_.max_pos_err) {
      const double scale = params_.max_pos_err / err_norm;
      ex *= scale;
      ey *= scale;
    }

    // world 系合成速度 = 前馈 + P 反馈
    const double vx_w = des_vx + params_.kp_pos * ex;
    const double vy_w = des_vy + params_.kp_pos * ey;

    // 旋转到机体系 (yaw 为机体相对 world 的偏航)
    const double c = std::cos(cur_yaw);
    const double s = std::sin(cur_yaw);
    cmd.vx = c * vx_w + s * vy_w;
    cmd.vy = -s * vx_w + c * vy_w;

    cmd.vyaw = des_yaw_dot +
               params_.kp_yaw * NormalizeAngle(des_yaw - cur_yaw);
    return cmd;
  }

 private:
  TrackerParams params_;
};

}  // namespace motionslam

#endif  // MOTIONSLAM_TRAJ_TRACKER_HPP_
