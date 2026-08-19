"""建图 (无头): 见 mapping_stack.launch.py (默认无 Foxglove / 无 forwarder).

  ros2 launch motionslam_bringup mapping.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory("motionslam_bringup")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "mapping_stack.launch.py")),
        ),
    ])
