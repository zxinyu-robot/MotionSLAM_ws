#include <algorithm>
#include <cmath>
#include <chrono>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

class MappingMotionGuard final : public rclcpp::Node {
 public:
  MappingMotionGuard() : Node("mapping_motion_guard") {
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.35);
    max_angular_speed_ = declare_parameter("max_angular_speed", 0.45);
    max_linear_acceleration_ = declare_parameter("max_linear_acceleration", 0.8);
    max_yaw_rate_change_ = declare_parameter("max_yaw_rate_change", 0.8);
    pause_on_violation_ = declare_parameter("pause_mapping_on_violation", true);

    ok_pub_ = create_publisher<std_msgs::msg::Bool>("/lio/mapping_motion_ok", 10);
    speed_pub_ = create_publisher<std_msgs::msg::Float32>("/lio/mapping_speed", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/robo/odom", 20,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { OnOdom(*msg); });
  }

 private:
  void OnOdom(const nav_msgs::msg::Odometry& msg) {
    const rclcpp::Time stamp(msg.header.stamp);
    const auto& v = msg.twist.twist.linear;
    const auto& w = msg.twist.twist.angular;
    const double speed = std::hypot(v.x, v.y);
    const double yaw_rate = std::abs(w.z);
    const double acceleration =
        previous_stamp_.nanoseconds() == 0
            ? 0.0
            : std::abs(speed - previous_speed_) /
                  std::max(1e-3, (stamp - previous_stamp_).seconds());
    const double yaw_rate_change =
        previous_stamp_.nanoseconds() == 0
            ? 0.0
            : std::abs(yaw_rate - previous_yaw_rate_) /
                  std::max(1e-3, (stamp - previous_stamp_).seconds());

    const bool violation = speed > max_linear_speed_ ||
                           yaw_rate > max_angular_speed_ ||
                           acceleration > max_linear_acceleration_ ||
                           yaw_rate_change > max_yaw_rate_change_;
    std_msgs::msg::Bool ok;
    ok.data = !pause_on_violation_ || !violation;
    ok_pub_->publish(ok);

    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(speed);
    speed_pub_->publish(speed_msg);

    if (violation && !last_violation_) {
      RCLCPP_WARN(get_logger(),
                  "mapping motion gate violated: speed=%.2f yaw_rate=%.2f "
                  "accel=%.2f yaw_rate_change=%.2f",
                  speed, yaw_rate, acceleration, yaw_rate_change);
    }
    last_violation_ = violation;
    previous_speed_ = speed;
    previous_yaw_rate_ = yaw_rate;
    previous_stamp_ = stamp;
  }

  double max_linear_speed_;
  double max_angular_speed_;
  double max_linear_acceleration_;
  double max_yaw_rate_change_;
  bool pause_on_violation_;
  bool last_violation_{false};
  double previous_speed_{0.0};
  double previous_yaw_rate_{0.0};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingMotionGuard>());
  rclcpp::shutdown();
  return 0;
}
