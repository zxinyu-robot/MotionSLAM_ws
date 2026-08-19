#ifndef MOTIONSLAM_VELOCITY_LIMITER_HPP_
#define MOTIONSLAM_VELOCITY_LIMITER_HPP_

#include <algorithm>

namespace motionslam {

/**
 * @brief 速度限幅器 (纯逻辑, 无 ROS 依赖, 便于单测)。
 *
 * NOTE: 限幅值须小于 Go2 避障服务自身上限, 与 config/pipeline.yaml 保持一致。
 */
struct VelocityLimits {
  double vx_min = -0.4;   ///< 后退速度下限 (m/s)
  double vx_max = 1.0;    ///< 前进速度上限 (m/s)
  double vy_abs = 0.4;    ///< 横移速度绝对值上限 (m/s)
  double vyaw_abs = 1.0;  ///< 角速度绝对值上限 (rad/s)
};

class VelocityLimiter {
 public:
  explicit VelocityLimiter(const VelocityLimits& limits) : limits_(limits) {}

  /**
   * @brief 对速度指令限幅。
   * @param vx   前向速度 (m/s), 输入输出
   * @param vy   横向速度 (m/s), 输入输出
   * @param vyaw 角速度 (rad/s), 输入输出
   *
   * 示例:
   * @code
   *   VelocityLimiter limiter({});
   *   double vx = 2.0, vy = 0.0, vyaw = 0.0;
   *   limiter.Clamp(vx, vy, vyaw);  // vx -> 1.0
   * @endcode
   */
  void Clamp(double& vx, double& vy, double& vyaw) const {
    vx = std::clamp(vx, limits_.vx_min, limits_.vx_max);
    vy = std::clamp(vy, -limits_.vy_abs, limits_.vy_abs);
    vyaw = std::clamp(vyaw, -limits_.vyaw_abs, limits_.vyaw_abs);
  }

 private:
  VelocityLimits limits_;
};

}  // namespace motionslam

#endif  // MOTIONSLAM_VELOCITY_LIMITER_HPP_
