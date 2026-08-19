"""在线 /lio/global_map → /map 2D 预览 (Foxglove Map 面板)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="motionslam_bringup",
            executable="cloud_to_grid.py",
            name="cloud_to_grid_preview",
            output="screen",
            parameters=[{
                "input_topic": "/lio/global_map",
                "output_topic": "/map",
                "resolution": 0.05,
                "width_m": 30.0,
                "height_m": 30.0,
                "z_min": 0.08,
                "z_max": 0.50,
                "center_on_robot": True,
                "inflation_cells": 2,
            }],
        ),
    ])
