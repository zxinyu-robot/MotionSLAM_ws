/**
 * Pose graph backend: keyframe odom edges, Scan Context + ICP loops, SE(2) optimize,
 * publish map->world correction (LIO world frame == historical /lio/odom frame).
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "motionslam/PoseGraphSe2.hpp"
#include "motionslam/ScanContext2D.hpp"

namespace {

using motionslam::ScanContext2D;
using motionslam::Se2Between;
using motionslam::Se2Compose;
using motionslam::Se2Edge;
using motionslam::Se2Inverse;
using motionslam::Se2Pose;
using motionslam::OptimizePoseGraph;

struct Keyframe {
  rclcpp::Time stamp;
  Se2Pose lio;       // raw LIO pose in world
  Se2Pose graph;     // optimized pose in map
  ScanContext2D sc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world;
};

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

geometry_msgs::msg::Transform ToTransform(const Se2Pose& t) {
  geometry_msgs::msg::Transform out;
  out.translation.x = t.x;
  out.translation.y = t.y;
  out.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, t.yaw);
  out.rotation.x = q.x();
  out.rotation.y = q.y();
  out.rotation.z = q.z();
  out.rotation.w = q.w();
  return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudMsgToPcl(
    const sensor_msgs::msg::PointCloud2& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelDown(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in, double leaf) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                 static_cast<float>(leaf));
  vg.filter(*out);
  return out;
}

void CloudToBodyVectors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& world,
                        const Se2Pose& pose, std::vector<float>& x,
                        std::vector<float>& y, std::vector<float>& z) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  x.clear();
  y.clear();
  z.clear();
  x.reserve(world->size());
  y.reserve(world->size());
  z.reserve(world->size());
  for (const auto& pt : world->points) {
    const double dx = pt.x - pose.x;
    const double dy = pt.y - pose.y;
    x.push_back(static_cast<float>(c * dx + s * dy));
    y.push_back(static_cast<float>(-s * dx + c * dy));
    z.push_back(pt.z);
  }
}

bool RunIcp(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
            const Eigen::Matrix4f& guess, Eigen::Matrix4f* out,
            double max_corr_dist) {
  if (target->size() < 80 || source->size() < 80) {
    return false;
  }
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(static_cast<float>(max_corr_dist));
  icp.setMaximumIterations(35);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-3);
  icp.setInputTarget(target);
  icp.setInputSource(source);
  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned, guess);
  if (!icp.hasConverged() || icp.getFitnessScore() > 0.35) {
    return false;
  }
  *out = icp.getFinalTransformation();
  return true;
}

Eigen::Matrix4f Se2ToMatrix4f(const Se2Pose& p) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(p.yaw));
  const float s = static_cast<float>(std::sin(p.yaw));
  T(0, 0) = c;
  T(0, 1) = -s;
  T(1, 0) = s;
  T(1, 1) = c;
  T(0, 3) = static_cast<float>(p.x);
  T(1, 3) = static_cast<float>(p.y);
  return T;
}

Se2Pose Matrix4fToSe2(const Eigen::Matrix4f& T) {
  Se2Pose p;
  p.x = T(0, 3);
  p.y = T(1, 3);
  p.yaw = std::atan2(T(1, 0), T(0, 0));
  return p;
}

double OdomPathLength(const std::vector<Keyframe>& kfs, std::size_t i,
                      std::size_t j) {
  if (j <= i) {
    return 0.0;
  }
  double len = 0.0;
  for (std::size_t k = i + 1; k <= j; ++k) {
    const Se2Pose d = Se2Between(kfs[k - 1].lio, kfs[k].lio);
    len += std::hypot(d.x, d.y);
  }
  return len;
}

}  // namespace

class PoseGraphBackend final : public rclcpp::Node {
 public:
  PoseGraphBackend() : Node("pose_graph_backend") {
    enabled_ = declare_parameter("enabled", true);
    map_frame_ = declare_parameter("map_frame", "map");
    world_frame_ = declare_parameter("world_frame", "world");
    correction_topic_ =
        declare_parameter("correction_topic", "/lio/map_to_odom");
    sc_dist_threshold_ = declare_parameter("scan_context_dist_threshold", 0.35);
    min_loop_keyframe_gap_ = declare_parameter("min_loop_keyframe_gap", 25);
    loop_icp_max_corr_ = declare_parameter("loop_icp_max_corr_m", 2.5);
    loop_trans_weight_ = declare_parameter("loop_trans_weight", 500.0);
    loop_rot_weight_ = declare_parameter("loop_rot_weight", 800.0);
    odom_trans_weight_ = declare_parameter("odom_trans_weight", 200.0);
    odom_rot_weight_ = declare_parameter("odom_rot_weight", 400.0);
    max_loop_correction_m_ = declare_parameter("max_loop_correction_m", 1.0);
    max_loop_correction_deg_ =
        declare_parameter("max_loop_correction_deg", 20.0);
    voxel_leaf_ = declare_parameter("keyframe_voxel_leaf", 0.25);
    sc_z_min_ = declare_parameter("scan_context_z_min", -0.5);
    sc_z_max_ = declare_parameter("scan_context_z_max", 1.5);
    optimize_every_n_keyframes_ =
        declare_parameter("optimize_every_n_keyframes", 3);
    min_loop_travel_m_ = declare_parameter("min_loop_travel_m", 2.0);
    max_loop_revisit_m_ = declare_parameter("max_loop_revisit_m", 1.5);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    correction_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(correction_topic_, 10);
    loop_pub_ = create_publisher<std_msgs::msg::Bool>("/lio/backend/loop_closed", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_cloud_ = msg;
        });
    keyframe_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lio/backend/keyframe_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          OnKeyframe(*msg);
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this] { PublishCorrection(); });

    RCLCPP_INFO(get_logger(),
                "pose_graph_backend enabled=%s map=%s world=%s",
                enabled_ ? "true" : "false", map_frame_.c_str(),
                world_frame_.c_str());
  }

 private:
  void OnKeyframe(const geometry_msgs::msg::PoseStamped& pose_msg) {
    if (!enabled_) {
      return;
    }
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cloud = latest_cloud_;
    }
    if (!cloud) {
      RCLCPP_WARN(get_logger(), "keyframe without cloud_world, skip");
      return;
    }

    Keyframe kf;
    kf.stamp = rclcpp::Time(pose_msg.header.stamp);
    kf.lio = PoseToSe2(pose_msg.pose);
    kf.graph = kf.lio;
    kf.cloud_world = VoxelDown(CloudMsgToPcl(*cloud), voxel_leaf_);

    std::vector<float> bx, by, bz;
    CloudToBodyVectors(kf.cloud_world, kf.lio, bx, by, bz);
    kf.sc.Build(bx, by, bz, sc_z_min_, sc_z_max_);

    const std::size_t idx = keyframes_.size();
    keyframes_.push_back(kf);

    if (idx > 0) {
      Se2Edge odom_edge;
      odom_edge.i = idx - 1;
      odom_edge.j = idx;
      odom_edge.meas = Se2Between(keyframes_[idx - 1].lio, kf.lio);
      odom_edge.trans_weight = odom_trans_weight_;
      odom_edge.rot_weight = odom_rot_weight_;
      odom_edge.is_loop = false;
      edges_.push_back(odom_edge);
    }

    TryLoopClosure(idx);

    if (idx > 0 && (idx % optimize_every_n_keyframes_ == 0 || loop_added_)) {
      OptimizeGraph();
      loop_added_ = false;
    }
    UpdateMapToWorld();
  }

  void TryLoopClosure(std::size_t j) {
    if (j < static_cast<std::size_t>(min_loop_keyframe_gap_)) {
      return;
    }
    double best_dist = 1e9;
    std::size_t best_i = 0;
    for (std::size_t i = 0; i + static_cast<std::size_t>(min_loop_keyframe_gap_) < j;
         ++i) {
      const double d = keyframes_[j].sc.Distance(keyframes_[i].sc);
      if (d < best_dist) {
        best_dist = d;
        best_i = i;
      }
    }
    if (best_dist > sc_dist_threshold_) {
      return;
    }

    const double travel = OdomPathLength(keyframes_, best_i, j);
    if (travel < min_loop_travel_m_) {
      return;
    }
    const double revisit = std::hypot(keyframes_[best_i].lio.x - keyframes_[j].lio.x,
                                      keyframes_[best_i].lio.y - keyframes_[j].lio.y);
    if (revisit > max_loop_revisit_m_) {
      return;
    }

    Eigen::Matrix4f init =
        Se2ToMatrix4f(Se2Between(keyframes_[best_i].lio, keyframes_[j].lio));
    Eigen::Matrix4f T_j_to_i;
    if (!RunIcp(keyframes_[best_i].cloud_world, keyframes_[j].cloud_world, init,
                &T_j_to_i, loop_icp_max_corr_)) {
      RCLCPP_INFO(get_logger(), "SC match i=%zu j=%zu but ICP reject (dist=%.3f)",
                  best_i, j, best_dist);
      return;
    }

    Se2Pose meas_ij = Matrix4fToSe2(T_j_to_i);
    const Se2Pose lio_ij = Se2Between(keyframes_[best_i].lio, keyframes_[j].lio);
    const double corr_yaw =
        std::abs(std::atan2(std::sin(meas_ij.yaw - lio_ij.yaw),
                            std::cos(meas_ij.yaw - lio_ij.yaw)));
    const double corr_m = std::hypot(meas_ij.x - lio_ij.x, meas_ij.y - lio_ij.y);
    if (corr_m > max_loop_correction_m_ ||
        corr_yaw > max_loop_correction_deg_ * M_PI / 180.0) {
      RCLCPP_WARN(get_logger(),
                  "loop i=%zu j=%zu rejected: corr too large (%.2fm %.1fdeg)",
                  best_i, j, corr_m, corr_yaw * 180.0 / M_PI);
      return;
    }

    Se2Edge loop;
    loop.i = best_i;
    loop.j = j;
    loop.meas = meas_ij;
    loop.trans_weight = loop_trans_weight_;
    loop.rot_weight = loop_rot_weight_;
    loop.is_loop = true;
    edges_.push_back(loop);
    loop_added_ = true;

    std_msgs::msg::Bool flag;
    flag.data = true;
    loop_pub_->publish(flag);
    RCLCPP_INFO(get_logger(),
                "loop closed i=%zu j=%zu sc=%.3f corr=(%.2fm, %.1fdeg)", best_i, j,
                best_dist, corr_m, corr_yaw * 180.0 / M_PI);
  }

  void OptimizeGraph() {
    if (keyframes_.size() < 2) {
      return;
    }
    std::vector<Se2Pose> nodes;
    nodes.reserve(keyframes_.size());
    for (const auto& kf : keyframes_) {
      nodes.push_back(kf.graph);
    }
    OptimizePoseGraph(nodes, edges_, 25);
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      keyframes_[i].graph = nodes[i];
    }
  }

  void UpdateMapToWorld() {
    if (keyframes_.empty()) {
      map_to_world_ = Se2Pose{};
      return;
    }
    const Keyframe& last = keyframes_.back();
    // T_map_world: compose map pose with inv(lio) -> correction at last keyframe
    map_to_world_ = Se2Compose(last.graph, Se2Inverse(last.lio));
  }

  void PublishCorrection() {
    if (!enabled_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = world_frame_;
    tf.transform = ToTransform(map_to_world_);
    tf_broadcaster_->sendTransform(tf);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf.header;
    pose.pose.position.x = map_to_world_.x;
    pose.pose.position.y = map_to_world_.y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, map_to_world_.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    correction_pub_->publish(pose);
  }

  bool enabled_{true};
  std::string map_frame_;
  std::string world_frame_;
  std::string correction_topic_;
  double sc_dist_threshold_;
  int min_loop_keyframe_gap_;
  double loop_icp_max_corr_;
  double loop_trans_weight_;
  double loop_rot_weight_;
  double odom_trans_weight_;
  double odom_rot_weight_;
  double max_loop_correction_m_;
  double max_loop_correction_deg_;
  double voxel_leaf_;
  double sc_z_min_;
  double sc_z_max_;
  int optimize_every_n_keyframes_;
  double min_loop_travel_m_;
  double max_loop_revisit_m_;
  bool loop_added_{false};

  Se2Pose map_to_world_;
  std::vector<Keyframe> keyframes_;
  std::vector<Se2Edge> edges_;
  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr correction_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loop_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseGraphBackend>());
  rclcpp::shutdown();
  return 0;
}
