/**
 * @file CmdVelForwarder.cpp
 * @brief /motion/command -> Go2 运动 API 转发节点。
 *
 * 设计要点:
 *  - 纯 ROS2 实现, 不链接 unitree_sdk2;
 *  - 默认 backend=sport: /api/sport/request (Move 1008), 启动时自动 SwitchJoystick +
 *    BalanceStand + FreeAvoid;
 *  - 看门狗: 超时未收到 /motion/command 则零速;
 *  - 退出/SIGINT 时强制 StopMove + 零速 Move (避免杀进程后狗继续走)。
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <motionslam_msgs/msg/motion_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>

#include "motionslam/VelocityLimiter.hpp"

namespace {

constexpr char kSportRequestTopic[] = "/api/sport/request";
// 与 sport 一致用 /api/...；rt/ 前缀在 ROS2 侧会变成无人订阅的独立话题
constexpr char kObstaclesAvoidRequestTopic[] = "/api/obstacles_avoid/request";

constexpr int64_t kSportApiBalanceStand = 1002;
constexpr int64_t kSportApiStopMove = 1003;
constexpr int64_t kSportApiMove = 1008;
constexpr int64_t kSportApiSwitchJoystick = 1027;
constexpr int64_t kSportApiFreeAvoid = 2048;

constexpr int64_t kAvoidApiSwitchSet = 1001;
constexpr int64_t kAvoidApiMove = 1003;
constexpr int64_t kAvoidApiUseRemoteCommandFromApi = 1004;

rclcpp::QoS UnitreeRequestQoS() {
  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  return qos;
}

}  // namespace

namespace motionslam {

class CmdVelForwarder : public rclcpp::Node {
 public:
  CmdVelForwarder() : Node("cmd_vel_forwarder") {
    VelocityLimits limits;
    limits.vx_min = declare_parameter("vx_min", limits.vx_min);
    limits.vx_max = declare_parameter("vx_max", limits.vx_max);
    limits.vy_abs = declare_parameter("vy_abs", limits.vy_abs);
    limits.vyaw_abs = declare_parameter("vyaw_abs", limits.vyaw_abs);
    limiter_ = std::make_unique<VelocityLimiter>(limits);

    watchdog_timeout_ms_ = declare_parameter("watchdog_timeout_ms", 400);
    backend_ = declare_parameter<std::string>("backend", "sport");
    sport_free_avoid_enabled_ = declare_parameter("sport_free_avoid_enabled", false);

    const char* topic = kSportRequestTopic;
    if (backend_ == "obstacles_avoid") {
      topic = kObstaclesAvoidRequestTopic;
      use_sport_move_ = false;
    } else if (backend_ != "sport") {
      RCLCPP_WARN(get_logger(),
                  "未知 backend=%s, 回退为 sport", backend_.c_str());
      backend_ = "sport";
    }

    request_pub_ =
        create_publisher<unitree_api::msg::Request>(topic, UnitreeRequestQoS());
    // SwitchJoystick / BalanceStand 必须走 Sport 话题；obstacles_avoid 亦然
    sport_pub_ =
        create_publisher<unitree_api::msg::Request>(kSportRequestTopic, UnitreeRequestQoS());
    motion_cmd_sub_ = create_subscription<motionslam_msgs::msg::MotionCommand>(
        "/motion/command", 10,
        [this](motionslam_msgs::msg::MotionCommand::SharedPtr msg) {
          OnMotionCommand(*msg);
        });

    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                        [this] { OnWatchdog(); });

    if (use_sport_move_) {
      init_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                      [this] { OnSportInit(); });
    } else {
      // 机身 obstacles_avoid 服务偶发晚于节点启动, 重复 enable 几次
      init_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                      [this] { OnAvoidInit(); });
    }

    RCLCPP_INFO(get_logger(), "CmdVelForwarder ready: /motion/command -> %s (backend=%s)",
                topic, backend_.c_str());
  }

  ~CmdVelForwarder() override { EmergencyStop("destructor"); }

  void EmergencyStop(const char* reason) {
    bool expected = false;
    if (!estop_done_.compare_exchange_strong(expected, true)) {
      return;
    }
    RCLCPP_WARN(get_logger(), "EmergencyStop (%s): StopMove + 零速 + 恢复手柄", reason);
    for (int i = 0; i < 5; ++i) {
      SendSportRequest(kSportApiStopMove, "");
      SendMove(0.0, 0.0, 0.0);
      rclcpp::sleep_for(std::chrono::milliseconds(30));
    }
    RestoreRemoteControl();
  }

  void RestoreRemoteControl() {
    for (int i = 0; i < 3; ++i) {
      SendAvoidRequest(kAvoidApiUseRemoteCommandFromApi,
                       R"({"is_remote_commands_from_api":false})");
      SendSportRequest(kSportApiSwitchJoystick, R"({"data":true})");
      SendSportRequest(kSportApiBalanceStand, "");
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_INFO(get_logger(), "已恢复手柄遥控 (SwitchJoystick=true, remote_from_api=false)");
  }

  void HandoffToApiControl() {
    for (int i = 0; i < 3; ++i) {
      SendSportRequest(kSportApiSwitchJoystick, R"({"data":false})");
      SendSportRequest(kSportApiBalanceStand, "");
      SendAvoidRequest(kAvoidApiSwitchSet, R"({"enable":true})");
      SendAvoidRequest(kAvoidApiUseRemoteCommandFromApi,
                       R"({"is_remote_commands_from_api":true})");
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_INFO(get_logger(),
                 "已切 API 控狗 (SwitchJoystick=false, obstacles_avoid remote_from_api=true)");
  }

 private:
  void OnSportInit() {
    switch (init_step_++) {
      case 0:
        SendSportRequest(kSportApiSwitchJoystick, R"({"data":false})");
        RCLCPP_INFO(get_logger(), "sport init: SwitchJoystick(false)");
        break;
      case 1:
        SendSportRequest(kSportApiBalanceStand, "");
        RCLCPP_INFO(get_logger(), "sport init: BalanceStand");
        break;
      case 2:
        if (sport_free_avoid_enabled_) {
          SendSportRequest(kSportApiFreeAvoid, R"({"data":true})");
          RCLCPP_INFO(get_logger(), "sport init: FreeAvoid(true)");
        } else {
          RCLCPP_INFO(get_logger(), "sport init: FreeAvoid skipped (sport_free_avoid_enabled=false)");
        }
        init_timer_->cancel();
        break;
      default:
        init_timer_->cancel();
        break;
    }
  }

  void OnAvoidInit() {
    SendSportRequest(kSportApiSwitchJoystick, R"({"data":false})");
    SendSportRequest(kSportApiBalanceStand, "");
    SendAvoidRequest(kAvoidApiSwitchSet, R"({"enable":true})");
    SendAvoidRequest(kAvoidApiUseRemoteCommandFromApi,
                     R"({"is_remote_commands_from_api":true})");
    RCLCPP_INFO(get_logger(),
                "obstacles_avoid init: SwitchJoystick(false) enable=true remote_from_api=true (%d/5)",
                init_step_ + 1);
    if (++init_step_ >= 5) {
      init_timer_->cancel();
      RCLCPP_WARN(get_logger(),
                  "obstacles_avoid: 已发送 enable×5。无手柄 walking 需 SwitchJoystick(false)");
    }
  }

  void OnMotionCommand(const motionslam_msgs::msg::MotionCommand & msg) {
    if (estop_done_.load()) {
      return;
    }
    if (!use_sport_move_ && !avoid_ready_) {
      HandoffToApiControl();
      avoid_ready_ = true;
    }
    last_cmd_time_ = now();
    double vx = msg.vx;
    double vy = msg.vy;
    double vyaw = msg.yaw_rate;
    limiter_->Clamp(vx, vy, vyaw);
    SendMove(vx, vy, vyaw);
  }

  void OnWatchdog() {
    if (estop_done_.load()) {
      return;
    }
    if (last_cmd_time_.nanoseconds() == 0) {
      return;
    }
    const auto elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1000000;
    if (elapsed_ms > watchdog_timeout_ms_ && !stopped_) {
      RCLCPP_WARN(get_logger(), "motion/command 超时 %ldms, 下发零速", elapsed_ms);
      SendMove(0.0, 0.0, 0.0);
      SendSportRequest(kSportApiStopMove, "");
      stopped_ = true;
    }
  }

  void SendMove(double vx, double vy, double vyaw) {
    if (use_sport_move_) {
      std::ostringstream json;
      json << R"({"x":)" << vx << R"(,"y":)" << vy << R"(,"z":)" << vyaw << "}";
      SendSportRequest(kSportApiMove, json.str());
    } else {
      std::ostringstream json;
      json << R"({"x":)" << vx << R"(,"y":)" << vy << R"(,"yaw":)" << vyaw
           << R"(,"mode":0})";
      SendAvoidRequest(kAvoidApiMove, json.str());
    }
    stopped_ = (vx == 0.0 && vy == 0.0 && vyaw == 0.0);
  }

  void SendSportRequest(int64_t api_id, const std::string& parameter) {
    unitree_api::msg::Request req;
    req.header.identity.id = ++request_id_;
    req.header.identity.api_id = api_id;
    req.parameter = parameter;
    sport_pub_->publish(req);
  }

  void SendAvoidRequest(int64_t api_id, const std::string& parameter) {
    if (use_sport_move_) {
      return;
    }
    unitree_api::msg::Request req;
    req.header.identity.id = ++request_id_;
    req.header.identity.api_id = api_id;
    req.parameter = parameter;
    request_pub_->publish(req);
  }

  void SendJsonRequest(int64_t api_id, const std::string& parameter) {
    if (use_sport_move_) {
      SendSportRequest(api_id, parameter);
    } else {
      SendAvoidRequest(api_id, parameter);
    }
  }

  std::unique_ptr<VelocityLimiter> limiter_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
  rclcpp::Subscription<motionslam_msgs::msg::MotionCommand>::SharedPtr motion_cmd_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  int64_t watchdog_timeout_ms_ = 400;
  int64_t request_id_ = 0;
  int init_step_ = 0;
  bool stopped_ = true;
  bool avoid_ready_ = false;
  bool use_sport_move_ = true;
  bool sport_free_avoid_enabled_{false};
  std::string backend_{"sport"};
  std::atomic<bool> estop_done_{false};
};

}  // namespace motionslam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motionslam::CmdVelForwarder>();
  rclcpp::on_shutdown([node]() {
    if (node) {
      node->EmergencyStop("rclcpp_shutdown");
    }
  });
  rclcpp::spin(node);
  node->EmergencyStop("spin_exit");
  node.reset();
  rclcpp::shutdown();
  return 0;
}
