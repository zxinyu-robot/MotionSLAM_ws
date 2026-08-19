#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>

class MappingKeyframeManager final : public rclcpp::Node {
 public:
  MappingKeyframeManager() : Node("mapping_keyframe_manager") {
    min_translation_ = declare_parameter("min_translation", 0.35);
    min_rotation_rad_ =
        declare_parameter("min_rotation_deg", 8.0) * M_PI / 180.0;
    min_interval_ = declare_parameter("min_interval_sec", 0.5);
    allow_interval_only_ =
        declare_parameter("allow_interval_only_keyframes", false);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/lio/backend/keyframes", rclcpp::QoS(10).transient_local());
    count_pub_ = create_publisher<std_msgs::msg::UInt64>("/lio/backend/keyframe_count", 10);
    event_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/backend/keyframe_pose", 10);
    gate_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/lio/mapping_motion_ok", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { motion_ok_ = msg->data; });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odom", 20,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { OnOdom(*msg); });
  }

 private:
  static double Yaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void OnOdom(const nav_msgs::msg::Odometry& msg) {
    const rclcpp::Time stamp(msg.header.stamp);
    if (!motion_ok_) {
      return;
    }
    const auto& p = msg.pose.pose.position;
    const double yaw = Yaw(msg.pose.pose.orientation);
    if (!has_last_) {
      Add(msg, yaw);
      return;
    }
    const double distance = std::hypot(p.x - last_x_, p.y - last_y_);
    const double rotation = std::abs(std::atan2(std::sin(yaw - last_yaw_),
                                                std::cos(yaw - last_yaw_)));
    const double elapsed = (stamp - last_stamp_).seconds();
    if (elapsed < min_interval_) {
      return;
    }
    const bool moved =
        distance >= min_translation_ || rotation >= min_rotation_rad_;
    if (moved || (allow_interval_only_ && elapsed >= min_interval_)) {
      Add(msg, yaw);
    }
  }

  void Add(const nav_msgs::msg::Odometry& msg, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.header.frame_id = "world";
    pose.pose = msg.pose.pose;
    path_.header = pose.header;
    path_.poses.push_back(pose);
    path_pub_->publish(path_);
    std_msgs::msg::UInt64 count;
    count.data = path_.poses.size();
    count_pub_->publish(count);
    event_pub_->publish(pose);
    const auto& p = pose.pose.position;
    last_x_ = p.x;
    last_y_ = p.y;
    last_yaw_ = yaw;
    last_stamp_ = rclcpp::Time(msg.header.stamp);
    has_last_ = true;
  }

  double min_translation_;
  double min_rotation_rad_;
  double min_interval_;
  bool allow_interval_only_;
  bool motion_ok_{true};
  bool has_last_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_yaw_{0.0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr count_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr event_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingKeyframeManager>());
  rclcpp::shutdown();
  return 0;
}
