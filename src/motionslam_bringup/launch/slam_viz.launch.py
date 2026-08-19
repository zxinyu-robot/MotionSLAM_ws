"""一键建图+可视化 (兼容旧流程): mapping_stack + Foxglove + 在线 2D 预览.

默认 **不** 加载 map_viz.pcd (缺文件会导致 pcd_to_pointcloud 退出).
有离线 PCD 时:
  ros2 launch motionslam_bringup slam_viz.launch.py with_static_pcd:=true

Foxglove:
  Fixed Frame: world
  /map, /lio/cloud_world, /lio/backend/keyframe_count, TF map→world
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("motionslam_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_static_pcd",
            default_value="false",
            description="true 时发布 map_viz.pcd → /map_cloud",
        ),
        DeclareLaunchArgument(
            "static_pcd_file",
            default_value=os.path.join(
                os.environ.get("MOTIONSLAM_MAPS", "/ws/maps"), "map_viz.pcd"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "mapping_stack.launch.py")),
            launch_arguments={
                "with_foxglove": "true",
                "with_grid_preview": "true",
                "with_static_pcd": LaunchConfiguration("with_static_pcd"),
                "static_pcd_file": LaunchConfiguration("static_pcd_file"),
            }.items(),
        ),
    ])
