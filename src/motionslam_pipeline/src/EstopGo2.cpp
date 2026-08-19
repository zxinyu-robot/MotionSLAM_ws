/**
 * @file EstopGo2.cpp
 * @brief 急停: /cmd_vel 零速 + Sport StopMove + obstacles_avoid Move(0) + Sport Move(0)
 *
 * 不依赖 cmd_vel_forwarder 是否存活。Sport 与 L1 双通道都发，避免只停 Sport 时避障栈状态错乱。
 * 用法: ros2 run motionslam_pipeline estop_go2
 */
#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>

namespace {

constexpr char kSportTopic[] = "/api/sport/request";
constexpr char kAvoidTopic[] = "/api/obstacles_avoid/request";
constexpr int64_t kSportApiStopMove = 1003;
constexpr int64_t kSportApiMove = 1008;
constexpr int64_t kAvoidApiMove = 1003;
constexpr int kBurstCount = 8;
constexpr int kIntervalMs = 40;

rclcpp::QoS UnitreeQos() {
  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  return qos;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("estop_go2");

  auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  auto sport_pub =
      node->create_publisher<unitree_api::msg::Request>(kSportTopic, UnitreeQos());
  auto avoid_pub =
      node->create_publisher<unitree_api::msg::Request>(kAvoidTopic, UnitreeQos());

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  geometry_msgs::msg::Twist zero;
  int64_t req_id = 0;

  for (int i = 0; i < kBurstCount; ++i) {
    cmd_pub->publish(zero);

    unitree_api::msg::Request stop;
    stop.header.identity.id = ++req_id;
    stop.header.identity.api_id = kSportApiStopMove;
    stop.parameter = "";
    sport_pub->publish(stop);

    unitree_api::msg::Request move0;
    move0.header.identity.id = ++req_id;
    move0.header.identity.api_id = kSportApiMove;
    move0.parameter = R"({"x":0.0,"y":0.0,"z":0.0})";
    sport_pub->publish(move0);

    unitree_api::msg::Request avoid0;
    avoid0.header.identity.id = ++req_id;
    avoid0.header.identity.api_id = kAvoidApiMove;
    avoid0.parameter = R"({"x":0.0,"y":0.0,"yaw":0.0,"mode":0})";
    avoid_pub->publish(avoid0);

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
  }

  RCLCPP_WARN(node->get_logger(),
              "estop_go2: StopMove + Sport/Avoid 零速 + /cmd_vel 零速 x%d",
              kBurstCount);

  node.reset();
  rclcpp::shutdown();
  return 0;
}
