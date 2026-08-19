"""发布离线可视化地图 PCD 到 /map_cloud (frame: world)。"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_file",
            default_value=os.path.join(
                os.environ.get("MOTIONSLAM_MAPS", "/ws/maps"), "map_viz.pcd"
            ),
            description="Foxglove 可视化 PCD",
        ),
        Node(
            package="pcl_ros",
            executable="pcd_to_pointcloud",
            name="map_static_publisher",
            output="screen",
            parameters=[{
                "file_name": LaunchConfiguration("pcd_file"),
                "tf_frame": "world",
                "interval": 5.0,
            }],
            remappings=[("cloud_pcd", "/map_cloud")],
        ),
    ])
