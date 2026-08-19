"""发布稠密可视化地图 PCD 到 /map_cloud (frame: world).

默认读 map_viz.pcd (由 scans 合并, leaf=0.2); 重定位仍用 map_reloc.pcd.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maps_dir = os.environ.get("MOTIONSLAM_MAPS", "/ws/maps")
    default_pcd = os.path.join(maps_dir, "map_viz.pcd")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_file",
            default_value=default_pcd,
            description="稠密可视化 PCD 路径",
        ),
        Node(
            package="pcl_ros",
            executable="pcd_to_pointcloud",
            name="map_viz_publisher",
            output="screen",
            parameters=[{
                "file_name": LaunchConfiguration("pcd_file"),
                "tf_frame": "world",
                "interval": 1.0,
            }],
            remappings=[("cloud_pcd", "/map_cloud")],
        ),
    ])
