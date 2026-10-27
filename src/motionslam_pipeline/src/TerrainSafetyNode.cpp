/**
 * @file TerrainSafetyNode.cpp
 * @brief 地形安全评估节点：/terrain/observation -> TerrainAssessor -> /terrain/decision。
 *
 * 感知（vkernel 高程图/真机 elevation map）产出 TerrainObservation；
 * 本节点只跑 legged_terrain::TerrainAssessor 并透传决策，不做控制——
 * 底层运动约束由 CmdVelForwarder 消费 safety_mode 施加。
 */

#include <chrono>
#include <memory>

#include <legged_terrain_state/state_contract.h>
#include <legged_terrain_state/terrain_assessor.h>
#include <motionslam_msgs/msg/terrain_decision.hpp>
#include <motionslam_msgs/msg/terrain_observation.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using motionslam_msgs::msg::TerrainDecision;
using motionslam_msgs::msg::TerrainObservation;

class TerrainSafetyNode : public rclcpp::Node {
 public:
  TerrainSafetyNode() : Node("terrain_safety_node") {
    legged_terrain::TerrainConfig cfg;
    cfg.caution_slope_deg = declare_parameter("caution_slope_deg", 12.0);
    cfg.critical_slope_deg = declare_parameter("critical_slope_deg", 25.0);
    cfg.caution_roughness_m = declare_parameter("caution_roughness_m", 0.04);
    cfg.critical_roughness_m = declare_parameter("critical_roughness_m", 0.10);
    cfg.minimum_support_confidence =
        declare_parameter("minimum_support_confidence", 0.55);
    cfg.minimum_support_points = static_cast<std::size_t>(
        declare_parameter<int>("minimum_support_points", 30));
    obs_timeout_ms_ =
        declare_parameter<int>("obs_timeout_ms", 1000);

    assessor_ = std::make_unique<legged_terrain::TerrainAssessor>(cfg);

    decision_pub_ = create_publisher<TerrainDecision>("/terrain/decision", 10);
    obs_sub_ = create_subscription<TerrainObservation>(
        "/terrain/observation", rclcpp::SensorDataQoS(),
        [this](const TerrainObservation::SharedPtr msg) { OnObservation(msg); });

    watchdog_ = create_wall_timer(
        std::chrono::milliseconds(200), [this] { CheckStale(); });

    RCLCPP_INFO(get_logger(),
                "terrain_safety_node up: caution=%.1fdeg critical=%.1fdeg",
                cfg.caution_slope_deg, cfg.critical_slope_deg);
  }

 private:
  void OnObservation(const TerrainObservation::SharedPtr& msg) {
    legged_terrain::TerrainObservation obs;
    obs.support_height_m = msg->support_height_m;
    obs.estimated_normal = {msg->surface_normal[0], msg->surface_normal[1],
                            msg->surface_normal[2]};
    obs.roughness_m = msg->roughness_m;
    obs.support_points = msg->support_points;
    obs.expected_points = msg->expected_points;
    obs.header.timestamp_ns =
        static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
        msg->header.stamp.nanosec;
    obs.header.frame_id = msg->header.frame_id;

    legged_terrain::TerrainDecision d;
    try {
      d = assessor_->evaluate(obs);
    } catch (const std::invalid_argument& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "invalid terrain observation: %s", e.what());
      return;
    }

    auto out = TerrainDecision();
    out.header.stamp = now();
    out.header.frame_id = msg->header.frame_id;
    out.faults = static_cast<std::uint32_t>(d.terrain.header.faults);
    out.risk = static_cast<std::uint8_t>(d.risk);
    out.safety_mode = static_cast<std::uint8_t>(d.safety_mode);
    out.support_height_m = d.terrain.support_height_m;
    out.surface_normal = d.terrain.surface_normal;
    out.slope_deg = d.terrain.slope_deg;
    out.roughness_m = d.terrain.roughness_m;
    out.support_confidence = d.terrain.support_confidence;
    out.traversability = d.terrain.traversability;
    decision_pub_->publish(out);
    last_obs_time_ = now();
  }

  void CheckStale() {
    if (last_obs_time_.nanoseconds() == 0) return;
    const double age_ms = (now() - last_obs_time_).seconds() * 1000.0;
    if (age_ms > obs_timeout_ms_ && !stale_published_) {
      // 观测超时：按 fail-safe 发 Critical/HoldPosition 帧，由消费端决定是否保持
      auto out = TerrainDecision();
      out.header.stamp = now();
      out.faults = static_cast<std::uint32_t>(legged_terrain::FaultFlag::Stale);
      out.risk = static_cast<std::uint8_t>(legged_terrain::RiskLevel::Critical);
      out.safety_mode =
          static_cast<std::uint8_t>(legged_terrain::SafetyMode::HoldPosition);
      out.support_confidence = 0.0;
      out.traversability = 0.0;
      decision_pub_->publish(out);
      stale_published_ = true;
      RCLCPP_WARN(get_logger(),
                  "terrain observation stale > %d ms, holding position",
                  obs_timeout_ms_);
    } else if (age_ms <= obs_timeout_ms_) {
      stale_published_ = false;
    }
  }

  std::unique_ptr<legged_terrain::TerrainAssessor> assessor_;
  rclcpp::Publisher<TerrainDecision>::SharedPtr decision_pub_;
  rclcpp::Subscription<TerrainObservation>::SharedPtr obs_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::Time last_obs_time_{0, 0, RCL_ROS_TIME};
  int obs_timeout_ms_{1000};
  bool stale_published_{false};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
