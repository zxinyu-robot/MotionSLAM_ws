/**
 * Demo 模式 NavigationContext：无 floor registry / 无 Global ICP 依赖。
 * 借鉴 Nav2 lifecycle：configure → activate → deactivate → cleanup。
 */
#include "motionslam/MapSessionManager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cctype>
#include <string>

namespace motionslam {

namespace {

enum class LifecycleState {
  kUnconfigured,
  kInactive,
  kActive,
  kFinalized,
};

const char* LifecycleStateName(LifecycleState state) {
  switch (state) {
    case LifecycleState::kUnconfigured:
      return "unconfigured";
    case LifecycleState::kInactive:
      return "inactive";
    case LifecycleState::kActive:
      return "active";
    case LifecycleState::kFinalized:
      return "finalized";
  }
  return "unknown";
}

}  // namespace

class DemoNavigationContextNode final : public rclcpp::Node {
 public:
  DemoNavigationContextNode() : Node("demo_navigation_context") {
    state_page_path_ = declare_parameter<std::string>(
        "state_page", "/dev/shm/motionslam_navigation_context");
    session_id_ = declare_parameter<std::string>("session_id", "demo_session");
    tile_id_ = declare_parameter<std::string>("tile_id", "main");
    map_id_ = declare_parameter<std::string>("map_id", "demo_live_map");
    bounds_half_extent_m_ =
        declare_parameter<double>("bounds_half_extent_m", 500.0);

    state_page_ = std::make_unique<NavigationContextStatePage>(state_page_path_);

    lifecycle_pub_ = create_publisher<std_msgs::msg::String>(
        "/demo/lifecycle/state", rclcpp::QoS(1).reliable().transient_local());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
        "navigation_context/ready",
        rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "navigation_context/status",
        rclcpp::QoS(1).reliable().transient_local());

    transition_sub_ = create_subscription<std_msgs::msg::String>(
        "/demo/lifecycle/transition", 10,
        [this](const std_msgs::msg::String& msg) {
          HandleTransition(msg.data);
        });

    timer_ = create_wall_timer(std::chrono::seconds(1),
                                [this]() { PublishObservation(); });

    RCLCPP_INFO(get_logger(),
                "Demo NavigationContext 就绪 (无 registry/ICP); state_page=%s",
                state_page_path_.c_str());
    PublishObservation();
  }

 private:
  void HandleTransition(const std::string& command) {
    const std::string normalized = ToLower(command);
    if (normalized == "configure") {
      lifecycle_ = LifecycleState::kInactive;
      context_.reason = "configured";
    } else if (normalized == "activate") {
      lifecycle_ = LifecycleState::kActive;
      context_.reason = "demo_scan_active";
    } else if (normalized == "deactivate") {
      lifecycle_ = LifecycleState::kInactive;
      context_.reason = "deactivated";
    } else if (normalized == "cleanup" || normalized == "shutdown") {
      lifecycle_ = LifecycleState::kFinalized;
      context_.reason = "cleanup";
    } else if (normalized == "autostart") {
      lifecycle_ = LifecycleState::kActive;
      context_.reason = "demo_scan_active";
    } else {
      RCLCPP_WARN(get_logger(), "未知 lifecycle 命令: %s", command.c_str());
      return;
    }
    RefreshContext();
    WriteStatePage();
    PublishObservation();
    RCLCPP_INFO(get_logger(), "lifecycle → %s (ready=%s)",
                LifecycleStateName(lifecycle_),
                context_.ready ? "true" : "false");
  }

  static std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
  }

  void RefreshContext() {
    context_.session_id = session_id_;
    context_.floor.building_id = "demo";
    context_.floor.floor_id = "live";
    context_.floor.map_id = map_id_;
    context_.floor.map_version = 1;
    context_.floor.tile_id = tile_id_;
    context_.floor.frame_id = "world";
    context_.floor.schema_version = 1;
    const double h = bounds_half_extent_m_;
    context_.floor.bounds = Bounds3d{-h, -h, -5.0, h, h, 5.0};
    context_.registry_valid = lifecycle_ == LifecycleState::kActive;
    context_.relocation_converged = lifecycle_ == LifecycleState::kActive;
    context_.relocation_fitness =
        lifecycle_ == LifecycleState::kActive ? 0.0
                                              : std::numeric_limits<double>::infinity();
    context_.max_fitness = 1.0;
    context_.ready = lifecycle_ == LifecycleState::kActive;
    if (lifecycle_ == LifecycleState::kUnconfigured) {
      context_.reason = "unconfigured";
      context_.ready = false;
      context_.registry_valid = false;
      context_.relocation_converged = false;
    }
  }

  void WriteStatePage() {
    std::string error;
    if (!state_page_->Write(context_, &error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "写 Demo NavigationContext 失败: %s", error.c_str());
    }
  }

  void PublishObservation() {
    RefreshContext();
    WriteStatePage();

    std_msgs::msg::String lifecycle_msg;
    lifecycle_msg.data = LifecycleStateName(lifecycle_);
    lifecycle_pub_->publish(lifecycle_msg);

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = context_.ready;
    ready_pub_->publish(ready_msg);

    std_msgs::msg::String status_msg;
    status_msg.data =
        std::string("{\"mode\":\"demo\",\"lifecycle\":\"") +
        LifecycleStateName(lifecycle_) + "\",\"ready\":" +
        (context_.ready ? "true" : "false") + ",\"relocation_converged\":" +
        (context_.relocation_converged ? "true" : "false") +
        ",\"reason\":\"" + context_.reason + "\"}";
    status_pub_->publish(status_msg);
  }

  std::string state_page_path_;
  std::string session_id_;
  std::string tile_id_;
  std::string map_id_;
  double bounds_half_extent_m_{500.0};
  LifecycleState lifecycle_{LifecycleState::kUnconfigured};
  NavigationContext context_;
  std::unique_ptr<NavigationContextStatePage> state_page_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr transition_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motionslam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motionslam::DemoNavigationContextNode>());
  rclcpp::shutdown();
  return 0;
}
