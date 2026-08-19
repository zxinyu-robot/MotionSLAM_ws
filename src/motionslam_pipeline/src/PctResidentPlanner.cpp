/**
 * @file PctResidentPlanner.cpp
 * @brief 常驻全局规划：C++ 体素缓存 + 2D A*，替代 Python/Open3D 热路径。
 *
 * 话题/服务与 Python PCT 对齐：explored_areas、state_estimation、
 * /goal_pose、/global_path、/build_tomogram。启动后点云够了即预热建图。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace {

struct VoxelKeyHash {
  std::size_t operator()(std::int64_t k) const noexcept {
    return static_cast<std::size_t>(k);
  }
};

std::int64_t VoxelKey(int ix, int iy, int iz) {
  return (static_cast<std::int64_t>(ix) & 0x1FFFFF) |
         ((static_cast<std::int64_t>(iy) & 0x1FFFFF) << 21) |
         ((static_cast<std::int64_t>(iz) & 0x1FFFFF) << 42);
}

struct GridAStar {
  int w = 0;
  int h = 0;
  std::vector<uint8_t> occ;  // 1 = blocked

  bool In(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
  bool Free(int x, int y) const { return In(x, y) && occ[y * w + x] == 0; }

  std::vector<std::pair<int, int>> Search(int sx, int sy, int gx, int gy) const {
    if (!Free(sx, sy) || !Free(gx, gy)) {
      return {};
    }
    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int n = w * h;
    std::vector<int> came(n, -1);
    std::vector<float> g(n, std::numeric_limits<float>::infinity());
    auto hfun = [&](int x, int y) {
      const float ax = static_cast<float>(x - gx);
      const float ay = static_cast<float>(y - gy);
      return std::sqrt(ax * ax + ay * ay);
    };
    using Node = std::pair<float, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    const int sidx = sy * w + sx;
    g[sidx] = 0.f;
    open.push({hfun(sx, sy), sidx});
    int steps = 0;
    while (!open.empty() && steps++ < n * 8) {
      const int cur = open.top().second;
      open.pop();
      const int cx = cur % w;
      const int cy = cur / w;
      if (cx == gx && cy == gy) {
        std::vector<std::pair<int, int>> path;
        for (int i = cur; i >= 0; i = came[i]) {
          path.emplace_back(i % w, i / w);
          if (i == sidx) {
            break;
          }
        }
        std::reverse(path.begin(), path.end());
        return path;
      }
      for (int k = 0; k < 8; ++k) {
        const int nx = cx + dx[k];
        const int ny = cy + dy[k];
        if (!Free(nx, ny)) {
          continue;
        }
        if (dx[k] != 0 && dy[k] != 0 &&
            (!Free(cx + dx[k], cy) || !Free(cx, cy + dy[k]))) {
          continue;
        }
        const int ni = ny * w + nx;
        const float step = (dx[k] != 0 && dy[k] != 0) ? 1.414f : 1.f;
        const float ng = g[cur] + step;
        if (ng + 1e-4f < g[ni]) {
          g[ni] = ng;
          came[ni] = cur;
          open.push({ng + hfun(nx, ny), ni});
        }
      }
    }
    return {};
  }
};

}  // namespace

class PctResidentPlanner : public rclcpp::Node {
 public:
  PctResidentPlanner() : Node("pct_planner") {
    voxel_m_ = declare_parameter("cloud_voxel_m", 0.15);
    max_voxels_ = declare_parameter("cloud_max_voxels", 200000);
    min_voxels_ = declare_parameter("cloud_min_points", 800);
    z_min_ = declare_parameter("obstacle_z_min", 0.32);
    z_max_ = declare_parameter("obstacle_z_max", 1.50);
    grid_res_ = declare_parameter("grid_resolution", 0.20);
    inflate_m_ = declare_parameter("inflate_m", 0.35);
    start_clear_m_ = declare_parameter("start_clear_m", 0.10);
    max_start_snap_m_ = declare_parameter("max_start_snap_m", 0.40);
    max_goal_snap_m_ = declare_parameter("max_goal_snap_m", 0.15);
    warmup_period_s_ = declare_parameter("warmup_period_s", 2.0);
    max_grid_ = declare_parameter("max_grid_cells", 220);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "explored_areas", rclcpp::SensorDataQoS(),
        std::bind(&PctResidentPlanner::OnCloud, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "state_estimation", rclcpp::SensorDataQoS(),
        std::bind(&PctResidentPlanner::OnOdom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&PctResidentPlanner::OnGoal, this, std::placeholders::_1));

    rclcpp::QoS latch(1);
    latch.transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", latch);
    build_srv_ = create_service<std_srvs::srv::Trigger>(
        "/build_tomogram",
        std::bind(&PctResidentPlanner::OnBuild, this, std::placeholders::_1,
                  std::placeholders::_2));
    warmup_timer_ = create_wall_timer(
        std::chrono::duration<double>(warmup_period_s_),
        std::bind(&PctResidentPlanner::OnWarmup, this));

    RCLCPP_INFO(get_logger(),
                "pct_resident_planner C++ ready voxel=%.2f min=%d (no Open3D)",
                voxel_m_, min_voxels_);
  }

 private:
  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::lock_guard<std::mutex> lock(mu_);
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const int ix = static_cast<int>(std::floor(p.x / voxel_m_));
      const int iy = static_cast<int>(std::floor(p.y / voxel_m_));
      const int iz = static_cast<int>(std::floor(p.z / voxel_m_));
      voxels_[VoxelKey(ix, iy, iz)] = {p.x, p.y, p.z};
      if (static_cast<int>(voxels_.size()) > max_voxels_) {
        voxels_.erase(voxels_.begin());
      }
    }
    ++cloud_frames_;
  }

  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    odom_ = *msg;
    have_odom_ = true;
  }

  void OnWarmup() {
    std::lock_guard<std::mutex> lock(mu_);
    if (warmed_ && grid_cloud_frames_ == cloud_frames_) {
      return;
    }
    if (static_cast<int>(voxels_.size()) < min_voxels_) {
      return;
    }
    if (RebuildGridLocked()) {
      warmed_ = true;
      grid_cloud_frames_ = cloud_frames_;
      RCLCPP_INFO(get_logger(),
                  "PCT online tomogram voxels=%zu grid=%dx%d frames=%d",
                  voxels_.size(), grid_.w, grid_.h, cloud_frames_);
    }
  }

  void OnBuild(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lock(mu_);
    if (static_cast<int>(voxels_.size()) < min_voxels_) {
      res->success = false;
      res->message = "not enough voxels";
      return;
    }
    const bool ok = RebuildGridLocked();
    warmed_ = ok;
    if (ok) {
      grid_cloud_frames_ = cloud_frames_;
    }
    res->success = ok;
    res->message = ok ? "cpp_tomogram_ready" : "grid_build_failed";
  }

  void OnGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    const std::string frame =
        msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
    if (!have_odom_) {
      RCLCPP_WARN(get_logger(), "PCT goal ignored: no odom");
      PublishPathLocked({}, frame, msg->header.stamp);
      return;
    }
    if (static_cast<int>(voxels_.size()) >= min_voxels_ &&
        (!warmed_ || grid_cloud_frames_ != cloud_frames_)) {
      warmed_ = RebuildGridLocked();
      if (warmed_) {
        grid_cloud_frames_ = cloud_frames_;
      }
    }
    if (grid_.w <= 0) {
      RCLCPP_WARN(get_logger(), "PCT goal ignored: empty grid");
      PublishPathLocked({}, frame, msg->header.stamp);
      return;
    }
    const double sx = odom_.pose.pose.position.x;
    const double sy = odom_.pose.pose.position.y;
    const double gx = msg->pose.position.x;
    const double gy = msg->pose.position.y;
    auto path = PlanLocked(sx, sy, gx, gy);
    PublishPathLocked(path, frame, msg->header.stamp);
  }

  bool RebuildGridLocked() {
    if (voxels_.empty()) {
      return false;
    }
    double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    for (const auto & kv : voxels_) {
      minx = std::min(minx, static_cast<double>(kv.second.x));
      miny = std::min(miny, static_cast<double>(kv.second.y));
      maxx = std::max(maxx, static_cast<double>(kv.second.x));
      maxy = std::max(maxy, static_cast<double>(kv.second.y));
    }
    origin_x_ = minx;
    origin_y_ = miny;
    int w = static_cast<int>(std::ceil((maxx - minx) / grid_res_)) + 3;
    int h = static_cast<int>(std::ceil((maxy - miny) / grid_res_)) + 3;
    w = std::clamp(w, 8, max_grid_);
    h = std::clamp(h, 8, max_grid_);
    grid_.w = w;
    grid_.h = h;
    grid_.occ.assign(static_cast<std::size_t>(w * h), 0);
    const int inflate = std::max(0, static_cast<int>(std::round(inflate_m_ / grid_res_)));
    for (const auto & kv : voxels_) {
      const auto & p = kv.second;
      if (p.z < z_min_ || p.z > z_max_) {
        continue;
      }
      const int gx = static_cast<int>(std::floor((p.x - origin_x_) / grid_res_));
      const int gy = static_cast<int>(std::floor((p.y - origin_y_) / grid_res_));
      for (int oy = -inflate; oy <= inflate; ++oy) {
        for (int ox = -inflate; ox <= inflate; ++ox) {
          const int x = gx + ox;
          const int y = gy + oy;
          if (grid_.In(x, y)) {
            grid_.occ[y * w + x] = 1;
          }
        }
      }
    }
    return true;
  }

  std::pair<int, int> WorldToGrid(double x, double y) const {
    return {static_cast<int>(std::floor((x - origin_x_) / grid_res_)),
            static_cast<int>(std::floor((y - origin_y_) / grid_res_))};
  }

  std::pair<int, int> SnapFree(int x, int y) const {
    if (grid_.Free(x, y)) {
      return {x, y};
    }
    for (int r = 1; r <= 8; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (grid_.Free(x + dx, y + dy)) {
            return {x + dx, y + dy};
          }
        }
      }
    }
    return {x, y};
  }

  void ClearDisk(double wx, double wy, double radius_m) {
    const int r = std::max(1, static_cast<int>(std::round(radius_m / grid_res_)));
    const auto [cx, cy] = WorldToGrid(wx, wy);
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) {
          continue;
        }
        const int x = cx + dx;
        const int y = cy + dy;
        if (grid_.In(x, y)) {
          grid_.occ[y * grid_.w + x] = 0;
        }
      }
    }
  }

  std::vector<std::pair<double, double>> PlanLocked(double sx, double sy, double gx,
                                                    double gy) {
    ClearDisk(sx, sy, start_clear_m_);
    auto [ix, iy] = SnapFree(WorldToGrid(sx, sy).first, WorldToGrid(sx, sy).second);
    auto [jx, jy] = SnapFree(WorldToGrid(gx, gy).first, WorldToGrid(gx, gy).second);
    const double snapped_sx = origin_x_ + (ix + 0.5) * grid_res_;
    const double snapped_sy = origin_y_ + (iy + 0.5) * grid_res_;
    const double snapped_gx = origin_x_ + (jx + 0.5) * grid_res_;
    const double snapped_gy = origin_y_ + (jy + 0.5) * grid_res_;
    if (std::hypot(snapped_sx - sx, snapped_sy - sy) > max_start_snap_m_ ||
        std::hypot(snapped_gx - gx, snapped_gy - gy) > max_goal_snap_m_) {
      return {};
    }
    const auto cells = grid_.Search(ix, iy, jx, jy);
    std::vector<std::pair<double, double>> xy;
    xy.reserve(cells.size());
    for (const auto & c : cells) {
      xy.emplace_back(origin_x_ + (c.first + 0.5) * grid_res_,
                      origin_y_ + (c.second + 0.5) * grid_res_);
    }
    return xy;
  }

  void PublishPathLocked(const std::vector<std::pair<double, double>> & xy,
                         const std::string & frame,
                         const builtin_interfaces::msg::Time & request_stamp) {
    nav_msgs::msg::Path path;
    path.header.stamp = request_stamp;
    path.header.frame_id = frame;
    for (const auto & p : xy) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.first;
      ps.pose.position.y = p.second;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "PCT C++ /global_path poses=%zu", path.poses.size());
  }

  double voxel_m_ = 0.15;
  int max_voxels_ = 200000;
  int min_voxels_ = 800;
  double z_min_ = 0.32;
  double z_max_ = 1.5;
  double grid_res_ = 0.2;
  double inflate_m_ = 0.35;
  double start_clear_m_ = 0.10;
  double max_start_snap_m_ = 0.40;
  double max_goal_snap_m_ = 0.15;
  double warmup_period_s_ = 2.0;
  int max_grid_ = 220;

  std::mutex mu_;
  std::unordered_map<std::int64_t, pcl::PointXYZ, VoxelKeyHash> voxels_;
  nav_msgs::msg::Odometry odom_;
  bool have_odom_ = false;
  bool warmed_ = false;
  int cloud_frames_ = 0;
  int grid_cloud_frames_ = 0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  GridAStar grid_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr build_srv_;
  rclcpp::TimerBase::SharedPtr warmup_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PctResidentPlanner>());
  rclcpp::shutdown();
  return 0;
}
