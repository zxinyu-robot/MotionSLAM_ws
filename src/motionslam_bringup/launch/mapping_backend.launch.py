"""建图位姿图后端: 运动门控 + 关键帧 + 在线 loop_detection (读 pipeline.yaml)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("motionslam_bringup")
    pipeline_yaml = os.path.join(bringup_share, "config", "pipeline.yaml")

    with_loop_detection = LaunchConfiguration("with_loop_detection")
    use_legacy_pose_graph = LaunchConfiguration("use_legacy_pose_graph")

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_loop_detection",
            default_value="true",
            description="启用 loop_detection_node (在线 SC + 异步 PGO)",
        ),
        DeclareLaunchArgument(
            "use_legacy_pose_graph",
            default_value="false",
            description="回退旧 pose_graph_backend (含 ICP)",
        ),
        Node(
            package="motionslam_pipeline",
            executable="mapping_motion_guard",
            name="mapping_motion_guard",
            output="screen",
            parameters=[pipeline_yaml],
        ),
        Node(
            package="motionslam_pipeline",
            executable="mapping_keyframe_manager",
            name="mapping_keyframe_manager",
            output="screen",
            parameters=[pipeline_yaml],
        ),
        Node(
            condition=IfCondition(with_loop_detection),
            package="motionslam_pipeline",
            executable="loop_detection_node",
            name="loop_detection_node",
            output="screen",
            parameters=[pipeline_yaml],
        ),
        Node(
            condition=IfCondition(use_legacy_pose_graph),
            package="motionslam_pipeline",
            executable="pose_graph_backend",
            name="pose_graph_backend",
            output="screen",
            parameters=[pipeline_yaml],
        ),
    ])
