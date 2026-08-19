/**
 * @file RestoreGo2Remote.cpp
 * @brief 恢复 Go2 手柄遥控: SwitchJoystick(true) + remote_from_api=false
 *
 * 导航栈 cmd_vel_forwarder 会 SwitchJoystick(false) 或 is_remote_commands_from_api=true,
 * 停栈后必须显式恢复, 否则手柄完全无效。
 * 用法: ros2 run motionslam_pipeline restore_go2_remote
 */
#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>

namespace {

constexpr char kSportTopic[] = "/api/sport/request";
constexpr char kAvoidTopic[] = "/api/obstacles_avoid/request";
constexpr int64_t kSportApiBalanceStand = 1002;
constexpr int64_t kSportApiSwitchJoystick = 1027;
constexpr int64_t kAvoidApiUseRemoteCommandFromApi = 1004;
constexpr int kBurstCount = 5;
constexpr int kIntervalMs = 80;

rclcpp::QoS UnitreeQos() {
  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  return qos;
}

void PublishRequest(const rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr& pub,
                    int64_t& req_id, int64_t api_id, const std::string& parameter) {
  unitree_api::msg::Request req;
  req.header.identity.id = ++req_id;
  req.header.identity.api_id = api_id;
  req.parameter = parameter;
  pub->publish(req);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("restore_go2_remote");

  auto sport_pub =
      node->create_publisher<unitree_api::msg::Request>(kSportTopic, UnitreeQos());
  auto avoid_pub =
      node->create_publisher<unitree_api::msg::Request>(kAvoidTopic, UnitreeQos());

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  int64_t req_id = 0;
  for (int i = 0; i < kBurstCount; ++i) {
    PublishRequest(avoid_pub, req_id, kAvoidApiUseRemoteCommandFromApi,
                   R"({"is_remote_commands_from_api":false})");
    PublishRequest(sport_pub, req_id, kSportApiSwitchJoystick, R"({"data":true})");
    PublishRequest(sport_pub, req_id, kSportApiBalanceStand, "");
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
  }

  RCLCPP_WARN(node->get_logger(),
              "restore_go2_remote: SwitchJoystick(true) + remote_from_api=false x%d",
              kBurstCount);

  node.reset();
  rclcpp::shutdown();
  return 0;
}
