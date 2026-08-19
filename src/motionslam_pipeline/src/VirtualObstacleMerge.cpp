/**
 * @file VirtualObstacleMerge.cpp
 * @brief 把 /perception/virtual_obstacles 并入 SCAN 用的点云（玻璃/BEV 虚拟墙）。
 */
#include <memory>
#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class VirtualObstacleMerge : public rclcpp::Node {
 public:
  VirtualObstacleMerge() : Node("virtual_obstacle_merge") {
    lidar_topic_ = declare_parameter("lidar_topic", "/lio/cloud_world");
    virtual_topic_ =
        declare_parameter("virtual_obstacle_topic", "/perception/virtual_obstacles");
    virtual_b_topic_ =
        declare_parameter("virtual_obstacle_topic_b", "/perception/virtual_obstacles_bev");
    out_topic_ = declare_parameter("output_topic", "/perception/scan_cloud_fused");

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VirtualObstacleMerge::OnLidar, this, std::placeholders::_1));
    virt_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        virtual_topic_, 10,
        std::bind(&VirtualObstacleMerge::OnVirtual, this, std::placeholders::_1));
    virt_b_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        virtual_b_topic_, 10,
        std::bind(&VirtualObstacleMerge::OnVirtualB, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "virtual_obstacle_merge %s + %s → %s", lidar_topic_.c_str(),
                virtual_topic_.c_str(), out_topic_.c_str());
  }

 private:
  void OnVirtual(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    virtual_ = *msg;
    have_virtual_ = true;
  }

  void OnVirtualB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    virtual_b_ = *msg;
    have_virtual_b_ = true;
  }

  void OnLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> fused;
    pcl::fromROSMsg(*msg, fused);
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (have_virtual_ && !virtual_.data.empty()) {
        pcl::PointCloud<pcl::PointXYZ> extra;
        pcl::fromROSMsg(virtual_, extra);
        fused += extra;
      }
      if (have_virtual_b_ && !virtual_b_.data.empty()) {
        pcl::PointCloud<pcl::PointXYZ> extra;
        pcl::fromROSMsg(virtual_b_, extra);
        fused += extra;
      }
    }
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(fused, out);
    out.header = msg->header;
    pub_->publish(out);
  }

  std::string lidar_topic_;
  std::string virtual_topic_;
  std::string virtual_b_topic_;
  std::string out_topic_;
  std::mutex mu_;
  sensor_msgs::msg::PointCloud2 virtual_;
  sensor_msgs::msg::PointCloud2 virtual_b_;
  bool have_virtual_ = false;
  bool have_virtual_b_ = false;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr virt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr virt_b_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualObstacleMerge>());
  rclcpp::shutdown();
  return 0;
}
