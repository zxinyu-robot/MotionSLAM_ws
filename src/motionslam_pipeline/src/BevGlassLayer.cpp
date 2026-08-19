/**
 * @file BevGlassLayer.cpp
 * @brief 真 BEV（TensorRT）玻璃/窗类 → /perception/virtual_obstacles。
 *
 * 无 engine 时不跑 Python stub 推理，只把已有 BEV 栅格里的玻璃类投到虚拟障碍。
 * engine_path 非空且文件存在时预留 TensorRT 接入点（当前未链 nvinfer）。
 */
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

class BevGlassLayer : public rclcpp::Node {
 public:
  BevGlassLayer() : Node("bev_glass_layer") {
    engine_path_ = declare_parameter("engine_path", "");
    grid_topic_ = declare_parameter("bev_grid_topic", "/semantic/bev_grid_world");
    virtual_topic_ =
        declare_parameter("virtual_obstacle_topic", "/perception/virtual_obstacles");
    occ_threshold_ = declare_parameter("glass_occ_threshold", 70);
    z_base_ = declare_parameter("wall_z_base_m", 0.20);
    z_height_ = declare_parameter("wall_height_m", 1.20);
    z_step_ = declare_parameter("wall_z_step_m", 0.15);

    const bool engine_ok =
        !engine_path_.empty() && std::filesystem::exists(engine_path_);
    if (engine_ok) {
      RCLCPP_WARN(get_logger(),
                  "TensorRT engine 已配置 (%s)，当前节点尚未链接 nvinfer；"
                  "玻璃虚拟障碍仍来自 BEV 栅格话题",
                  engine_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "bev_glass_layer: 无 TensorRT engine，不使用 Python stub。"
                  "订阅 %s 中占用格作为玻璃/窗虚拟障碍",
                  grid_topic_.c_str());
    }

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        grid_topic_, 10,
        std::bind(&BevGlassLayer::OnGrid, this, std::placeholders::_1));
    virt_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(virtual_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/semantic/bev_status", 10);
    std_msgs::msg::String st;
    st.data = engine_ok ? "tensorrt_pending" : "grid_passthrough_no_stub";
    status_pub_->publish(st);
  }

 private:
  void OnGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);
    const double res = msg->info.resolution;
    const double ox = msg->info.origin.position.x;
    const double oy = msg->info.origin.position.y;
    std::vector<std::array<float, 3>> pts;
    pts.reserve(256);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int8_t v = msg->data[y * w + x];
        if (v < occ_threshold_) {
          continue;
        }
        const float wx = static_cast<float>(ox + (x + 0.5) * res);
        const float wy = static_cast<float>(oy + (y + 0.5) * res);
        for (double z = z_base_; z <= z_base_ + z_height_; z += z_step_) {
          pts.push_back({wx, wy, static_cast<float>(z)});
        }
      }
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    if (cloud.header.frame_id.empty()) {
      cloud.header.frame_id = "world";
    }
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & p : pts) {
      *iter_x = p[0];
      *iter_y = p[1];
      *iter_z = p[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    virt_pub_->publish(cloud);
  }

  std::string engine_path_;
  std::string grid_topic_;
  std::string virtual_topic_;
  int occ_threshold_ = 70;
  double z_base_ = 0.2;
  double z_height_ = 1.2;
  double z_step_ = 0.15;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr virt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BevGlassLayer>());
  rclcpp::shutdown();
  return 0;
}
