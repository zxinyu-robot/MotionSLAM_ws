/**
 * Online loop detection: SC retrieval + async pose-only PGO.
 * Does not modify Super-LIO OctVox; publishes map->world TF and pgo_state.
 */
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "motionslam/PoseGraphSe2.hpp"
#include "motionslam/ScanContext2D.hpp"

namespace {

using motionslam::OptimizePoseGraph;
using motionslam::ScanContext2D;
using motionslam::Se2Between;
using motionslam::Se2Compose;
using motionslam::Se2Edge;
using motionslam::Se2Inverse;
using motionslam::Se2Pose;

struct Keyframe {
  rclcpp::Time stamp;
  Se2Pose lio;
  Se2Pose graph;
  ScanContext2D sc;
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

Se2Pose LerpSe2(const Se2Pose& a, const Se2Pose& b, double alpha) {
  Se2Pose out;
  out.x = a.x + alpha * (b.x - a.x);
  out.y = a.y + alpha * (b.y - a.y);
  double dyaw = b.yaw - a.yaw;
  while (dyaw > M_PI) {
    dyaw -= 2.0 * M_PI;
  }
  while (dyaw <= -M_PI) {
    dyaw += 2.0 * M_PI;
  }
  out.yaw = a.yaw + alpha * dyaw;
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

class LoopDetectionNode final : public rclcpp::Node {
 public:
  LoopDetectionNode() : Node("loop_detection_node") {
    enabled_ = declare_parameter("enabled", true);
    map_frame_ = declare_parameter("map_frame", "map");
    world_frame_ = declare_parameter("world_frame", "world");
    correction_topic_ =
        declare_parameter("correction_topic", "/lio/map_to_odom");
    sc_dist_threshold_ = declare_parameter("scan_context_dist_threshold", 0.35);
    min_loop_keyframe_gap_ = declare_parameter("min_loop_keyframe_gap", 25);
    loop_trans_weight_ = declare_parameter("loop_trans_weight", 500.0);
    loop_rot_weight_ = declare_parameter("loop_rot_weight", 800.0);
    odom_trans_weight_ = declare_parameter("odom_trans_weight", 200.0);
    odom_rot_weight_ = declare_parameter("odom_rot_weight", 400.0);
    max_loop_correction_m_ = declare_parameter("max_loop_correction_m", 0.3);
    max_loop_correction_deg_ =
        declare_parameter("max_loop_correction_deg", 12.0);
    voxel_leaf_ = declare_parameter("keyframe_voxel_leaf", 0.25);
    sc_z_min_ = declare_parameter("scan_context_z_min", -0.5);
    sc_z_max_ = declare_parameter("scan_context_z_max", 1.5);
    min_loop_travel_m_ = declare_parameter("min_loop_travel_m", 2.0);
    max_loop_revisit_m_ = declare_parameter("max_loop_revisit_m", 1.5);
    tf_smooth_duration_s_ = declare_parameter("tf_smooth_duration_s", 0.35);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    correction_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(correction_topic_, 10);
    loop_pub_ =
        create_publisher<std_msgs::msg::Bool>("/lio/backend/loop_closed", 10);
    pgo_state_pub_ =
        create_publisher<std_msgs::msg::String>("/lio/backend/pgo_state", 10);
    drift_pub_ = create_publisher<std_msgs::msg::String>(
        "/lio/backend/drift_estimate", 10);
    generation_pub_ = create_publisher<std_msgs::msg::UInt64>(
        "/lio/backend/context_generation", 10);
    keyframe_poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "/lio/backend/keyframe_poses", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(cloud_mutex_);
          latest_cloud_ = msg;
        });
    keyframe_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lio/backend/keyframe_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          OnKeyframe(*msg);
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               [this] { OnTimer(); });

    pgo_thread_ = std::thread([this] { PgoWorker(); });

    PublishPgoState("IDLE");
    RCLCPP_INFO(get_logger(),
                "loop_detection_node enabled=%s map=%s world=%s",
                enabled_ ? "true" : "false", map_frame_.c_str(),
                world_frame_.c_str());
  }

  ~LoopDetectionNode() override {
    shutdown_ = true;
    pgo_cv_.notify_all();
    if (pgo_thread_.joinable()) {
      pgo_thread_.join();
    }
  }

 private:
  void OnKeyframe(const geometry_msgs::msg::PoseStamped& pose_msg) {
    if (!enabled_) {
      return;
    }
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
    }
    if (!cloud) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "keyframe without cloud_world, skip");
      return;
    }

    Keyframe kf;
    kf.stamp = rclcpp::Time(pose_msg.header.stamp);
    kf.lio = PoseToSe2(pose_msg.pose);
    kf.graph = kf.lio;

    const auto cloud_down = VoxelDown(CloudMsgToPcl(*cloud), voxel_leaf_);
    std::vector<float> bx, by, bz;
    CloudToBodyVectors(cloud_down, kf.lio, bx, by, bz);
    kf.sc.Build(bx, by, bz, sc_z_min_, sc_z_max_);

    std::size_t idx = 0;
    bool loop_added = false;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      idx = keyframes_.size();
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

      loop_added = TryLoopClosure(idx);
    }

    if (loop_added) {
      std::lock_guard<std::mutex> lock(pgo_mutex_);
      pgo_pending_ = true;
      pgo_cv_.notify_one();
    }
  }

  bool TryLoopClosure(std::size_t j) {
    if (j < static_cast<std::size_t>(min_loop_keyframe_gap_)) {
      return false;
    }

    double best_dist = 1e9;
    std::size_t best_i = 0;
    const auto& kf_j = keyframes_[j];

    for (std::size_t i = 0; i + static_cast<std::size_t>(min_loop_keyframe_gap_) < j;
         ++i) {
      const double revisit =
          std::hypot(keyframes_[i].lio.x - kf_j.lio.x,
                     keyframes_[i].lio.y - kf_j.lio.y);
      if (revisit > max_loop_revisit_m_) {
        continue;
      }
      const double d = kf_j.sc.Distance(keyframes_[i].sc);
      if (d < best_dist) {
        best_dist = d;
        best_i = i;
      }
    }

    if (best_dist > sc_dist_threshold_) {
      return false;
    }

    const double travel = OdomPathLength(keyframes_, best_i, j);
    if (travel < min_loop_travel_m_) {
      return false;
    }

    const Se2Pose lio_ij = Se2Between(keyframes_[best_i].lio, keyframes_[j].lio);
    const double corr_m = std::hypot(lio_ij.x, lio_ij.y);
    const double corr_yaw = std::abs(lio_ij.yaw);
    if (corr_m > max_loop_correction_m_ ||
        corr_yaw > max_loop_correction_deg_ * M_PI / 180.0) {
      RCLCPP_WARN(get_logger(),
                  "loop i=%zu j=%zu rejected: corr too large (%.2fm %.1fdeg)",
                  best_i, j, corr_m, corr_yaw * 180.0 / M_PI);
      return false;
    }

    Se2Edge loop;
    loop.i = best_i;
    loop.j = j;
    loop.meas = lio_ij;
    loop.trans_weight = loop_trans_weight_;
    loop.rot_weight = loop_rot_weight_;
    loop.is_loop = true;
    edges_.push_back(loop);

    std_msgs::msg::Bool flag;
    flag.data = true;
    loop_pub_->publish(flag);
    RCLCPP_INFO(get_logger(),
                "loop candidate i=%zu j=%zu sc=%.3f travel=%.1fm revisit=%.2fm",
                best_i, j, best_dist, travel,
                std::hypot(keyframes_[best_i].lio.x - kf_j.lio.x,
                           keyframes_[best_i].lio.y - kf_j.lio.y));
    return true;
  }

  void PgoWorker() {
    while (!shutdown_) {
      {
        std::unique_lock<std::mutex> lock(pgo_mutex_);
        pgo_cv_.wait(lock, [this] { return shutdown_ || pgo_pending_; });
        if (shutdown_) {
          break;
        }
        pgo_pending_ = false;
      }

      PublishPgoState("OPTIMIZING");

      Se2Pose target_map_to_world;
      double drift_m = 0.0;
      double drift_yaw_deg = 0.0;
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        if (keyframes_.size() < 2) {
          PublishPgoState("IDLE");
          continue;
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

        const Keyframe& last = keyframes_.back();
        target_map_to_world = Se2Compose(last.graph, Se2Inverse(last.lio));
        drift_m = std::hypot(target_map_to_world.x, target_map_to_world.y);
        drift_yaw_deg = target_map_to_world.yaw * 180.0 / M_PI;
      }

      {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_interp_start_ = displayed_map_to_world_;
        tf_interp_target_ = target_map_to_world;
        tf_interp_start_time_ = now();
        tf_interp_active_ = true;
      }

      context_generation_++;
      PublishKeyframePoses();
      PublishDriftEstimate(drift_m, drift_yaw_deg);
      PublishGeneration();
      PublishPgoState("APPLIED");
    }
  }

  void OnTimer() {
    if (!enabled_) {
      return;
    }

    Se2Pose display = displayed_map_to_world_;
    {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      if (tf_interp_active_) {
        const double elapsed = (now() - tf_interp_start_time_).seconds();
        const double alpha =
            std::min(1.0, elapsed / std::max(0.05, tf_smooth_duration_s_));
        display = LerpSe2(tf_interp_start_, tf_interp_target_, alpha);
        displayed_map_to_world_ = display;
        if (alpha >= 1.0) {
          tf_interp_active_ = false;
          displayed_map_to_world_ = tf_interp_target_;
          display = displayed_map_to_world_;
        }
      } else {
        std::lock_guard<std::mutex> g(graph_mutex_);
        if (!keyframes_.empty()) {
          const Keyframe& last = keyframes_.back();
          displayed_map_to_world_ =
              Se2Compose(last.graph, Se2Inverse(last.lio));
          display = displayed_map_to_world_;
        }
      }
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = world_frame_;
    tf.transform = ToTransform(display);
    tf_broadcaster_->sendTransform(tf);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf.header;
    pose.pose.position.x = display.x;
    pose.pose.position.y = display.y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, display.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    correction_pub_->publish(pose);
  }

  void PublishPgoState(const std::string& state) {
    std_msgs::msg::String msg;
    msg.data = state;
    pgo_state_pub_->publish(msg);
  }

  void PublishDriftEstimate(double trans_m, double yaw_deg) {
    std_msgs::msg::String msg;
    msg.data = "{\"trans_m\":" + std::to_string(trans_m) +
               ",\"yaw_deg\":" + std::to_string(yaw_deg) +
               ",\"confidence\":0.85}";
    drift_pub_->publish(msg);
  }

  void PublishGeneration() {
    std_msgs::msg::UInt64 msg;
    msg.data = context_generation_;
    generation_pub_->publish(msg);
  }

  void PublishKeyframePoses() {
    geometry_msgs::msg::PoseArray arr;
    arr.header.stamp = now();
    arr.header.frame_id = map_frame_;
    std::lock_guard<std::mutex> lock(graph_mutex_);
    arr.poses.reserve(keyframes_.size());
    for (const auto& kf : keyframes_) {
      geometry_msgs::msg::Pose p;
      p.position.x = kf.graph.x;
      p.position.y = kf.graph.y;
      p.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, kf.graph.yaw);
      p.orientation = tf2::toMsg(q);
      arr.poses.push_back(p);
    }
    keyframe_poses_pub_->publish(arr);
  }

  bool enabled_{true};
  std::string map_frame_;
  std::string world_frame_;
  std::string correction_topic_;
  double sc_dist_threshold_;
  int min_loop_keyframe_gap_;
  double loop_trans_weight_;
  double loop_rot_weight_;
  double odom_trans_weight_;
  double odom_rot_weight_;
  double max_loop_correction_m_;
  double max_loop_correction_deg_;
  double voxel_leaf_;
  double sc_z_min_;
  double sc_z_max_;
  double min_loop_travel_m_;
  double max_loop_revisit_m_;
  double tf_smooth_duration_s_;

  Se2Pose displayed_map_to_world_{};
  Se2Pose tf_interp_start_{};
  Se2Pose tf_interp_target_{};
  rclcpp::Time tf_interp_start_time_{0, 0, RCL_ROS_TIME};
  bool tf_interp_active_{false};
  std::uint64_t context_generation_{0};

  std::vector<Keyframe> keyframes_;
  std::vector<Se2Edge> edges_;
  std::mutex graph_mutex_;
  std::mutex cloud_mutex_;
  std::mutex tf_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  std::thread pgo_thread_;
  std::mutex pgo_mutex_;
  std::condition_variable pgo_cv_;
  bool pgo_pending_{false};
  std::atomic<bool> shutdown_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr correction_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pgo_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drift_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr generation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr keyframe_poses_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
