"""狗身位姿可视化: /lio/pose + /lio/robot_marker + TF base_link"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    prefix = get_package_prefix("motionslam_bringup")
    script = os.path.join(prefix, "lib", "motionslam_bringup", "robot_pose_viz.py")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        ExecuteProcess(
            cmd=[
                "python3", script,
                "--ros-args",
                "-p", "input_topic:=/lio/robo/odom",
                "-p", "level_orientation:=true",
                "-p", PythonExpression(["'use_sim_time:=' + '", use_sim_time, "'"]),
            ],
            output="screen",
        ),
    ])
