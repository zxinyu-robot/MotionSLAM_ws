"""Go2 电机监控 (导航时可与 Foxglove 联用)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("motionslam_bringup")
    pipeline_yaml = os.path.join(bringup_share, "config", "pipeline.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("pipeline_config", default_value=pipeline_yaml),
        Node(
            package="motionslam_bringup",
            executable="motor_monitor_node.py",
            name="motor_monitor_node",
            output="screen",
            parameters=[LaunchConfiguration("pipeline_config")],
        ),
    ])
