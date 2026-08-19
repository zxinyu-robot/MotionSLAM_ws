"""建图试走 (慢走 + L1 避障): mapping_stack + Foxglove + 预览 + obstacles_avoid.

容器内:
  ros2 launch motionslam_bringup mapping_walk.launch.py

配合:
  python3 /ws/scripts/tools/walk_odom_distance.py --distance 3 --speed 0.25
  bash /ws/scripts/tools/watch_mapping_backend.sh
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
            launch_arguments={
                "with_foxglove": "true",
                "with_grid_preview": "true",
                "with_forwarder": "true",
                "forwarder_backend": "obstacles_avoid",
                "with_static_pcd": "false",
            }.items(),
        ),
    ])
