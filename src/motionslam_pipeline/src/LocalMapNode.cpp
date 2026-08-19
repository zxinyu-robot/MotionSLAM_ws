/**
 * Projects LIO cloud_world (world frame) into map frame using T_mw = inv(T_map_world).
 */
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "motionslam/PoseGraphSe2.hpp"

namespace {

using motionslam::Se2Compose;
using motionslam::Se2Inverse;
using motionslam::Se2Pose;

Se2Pose PoseToSe2(const geometry_msgs::msg::Pose& p) {
  Se2Pose out;
  out.x = p.position.x;
  out.y = p.position.y;
  tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z,
                    p.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  out.yaw = yaw;
  return out;
}

Se2Pose OdomToSe2(const nav_msgs::msg::Odometry& odom) {
  return PoseToSe2(odom.pose.pose);
}

geometry_msgs::msg::Pose Se2ToPose(const Se2Pose& t) {
  geometry_msgs::msg::Pose out;
  out.position.x = t.x;
  out.position.y = t.y;
  out.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, t.yaw);
  out.orientation = tf2::toMsg(q);
  return out;
}

}  // namespace

class LocalMapNode final : public rclcpp::Node {
 public:
  LocalMapNode() : Node("local_map_node") {
    map_frame_ = declare_parameter("map_frame", "map");
    world_frame_ = declare_parameter("world_frame", "world");
    publish_hz_ = declare_parameter("publish_hz", 10.0);
    require_stance_ok_ = declare_parameter("require_stance_ok", true);

    cloud_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/local_map/cloud", 10);
    body_pose_pub_ =
        create_publisher<nav_msgs::msg::Odometry>("/local_map/body_pose", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_cloud_ = msg;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/robo/odom", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_odom_ = msg;
        });
    map_to_world_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lio/map_to_odom", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          map_to_world_ = PoseToSe2(msg->pose);
          has_map_to_world_ = true;
        });
    stance_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/local_map/stance_ok", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) {
          stance_ok_ = msg->data;
        });

    const double period = 1.0 / std::max(1.0, publish_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(period),
        [this] { PublishLocalMap(); });

    RCLCPP_INFO(get_logger(),
                "local_map_node map=%s world=%s hz=%.1f stance_gate=%s",
                map_frame_.c_str(), world_frame_.c_str(), publish_hz_,
                require_stance_ok_ ? "true" : "false");
  }

 private:
  void PublishLocalMap() {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    nav_msgs::msg::Odometry::SharedPtr odom;
    Se2Pose t_map_world;
    bool has_tf = false;
    bool stance = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cloud = latest_cloud_;
      odom = latest_odom_;
      if (has_map_to_world_) {
        t_map_world = map_to_world_;
        has_tf = true;
      }
      stance = stance_ok_;
    }

    if (!cloud || !odom) {
      return;
    }
    if (require_stance_ok_ && !stance) {
      return;
    }

    Se2Pose t_mw;
    if (has_tf) {
      t_mw = Se2Inverse(t_map_world);
    } else {
      t_mw = Se2Pose{};
    }

    const double c = std::cos(t_mw.yaw);
    const double s = std::sin(t_mw.yaw);

    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*cloud, input);
    pcl::PointCloud<pcl::PointXYZ> output;
    output.reserve(input.size());
    for (const auto& pt : input.points) {
      pcl::PointXYZ out;
      out.x = static_cast<float>(c * pt.x - s * pt.y + t_mw.x);
      out.y = static_cast<float>(s * pt.x + c * pt.y + t_mw.y);
      out.z = pt.z;
      output.push_back(out);
    }

    sensor_msgs::msg::PointCloud2 out_cloud;
    pcl::toROSMsg(output, out_cloud);
    out_cloud.header.stamp = cloud->header.stamp;
    out_cloud.header.frame_id = map_frame_;
    cloud_pub_->publish(out_cloud);

    const Se2Pose body_world = OdomToSe2(*odom);
    const Se2Pose body_map = Se2Compose(t_mw, body_world);

    nav_msgs::msg::Odometry body_map_odom;
    body_map_odom.header.stamp = odom->header.stamp;
    body_map_odom.header.frame_id = map_frame_;
    body_map_odom.child_frame_id = odom->child_frame_id;
    body_map_odom.pose.pose = Se2ToPose(body_map);
    body_map_odom.twist = odom->twist;
    body_pose_pub_->publish(body_map_odom);
  }

  std::string map_frame_;
  std::string world_frame_;
  double publish_hz_{10.0};
  bool require_stance_ok_{true};

  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  Se2Pose map_to_world_{};
  bool has_map_to_world_{false};
  bool stance_ok_{true};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr map_to_world_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stance_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalMapNode>());
  rclcpp::shutdown();
  return 0;
}
