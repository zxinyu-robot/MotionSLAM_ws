"""mid360 驱动 (阶段2 T2.2): 出 /livox/lidar + /livox/imu

网络配置用本包 config/MID360_config.json (host .18 / lidar .20),
不修改 livox_ros_driver2 子模块内的默认配置。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_json = os.path.join(
        get_package_share_directory("motionslam_bringup"),
        "config", "MID360_config.json")

    return LaunchDescription([
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                # xfer_format=1: livox 自定义点云格式 (Super-LIO lidar_type=1 要求)
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": config_json,
            }],
        ),
    ])
