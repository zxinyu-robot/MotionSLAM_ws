"""规范建图栈: 传感器 + Super-LIO 建图 + 后端 + 可选 Foxglove / 预览栅格 / 遥控下发.

推荐入口 (容器内):
  # 仅建图 (无头)
  ros2 launch motionslam_bringup mapping_stack.launch.py

  # 建图 + Foxglove + 在线 2D 预览 (默认不播 map_viz.pcd, 避免缺文件崩溃)
  ros2 launch motionslam_bringup mapping_stack.launch.py \\
      with_foxglove:=true with_grid_preview:=true

  # 建图 + 可视化 + 机身 L1 避障转发 (试走 / walk_odom_distance.py)
  ros2 launch motionslam_bringup mapping_stack.launch.py \\
      with_foxglove:=true with_grid_preview:=true with_forwarder:=true \\
      forwarder_backend:=obstacles_avoid

  # 等价旧名
  ros2 launch motionslam_bringup slam_viz.launch.py   # 薄封装, 见 slam_viz.launch.py

Foxglove Fixed Frame: world (闭环校正可看 TF map→world)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("motionslam_bringup")
    pipeline_yaml = os.path.join(bringup_share, "config", "pipeline.yaml")
    mapping_lio_yaml = os.path.join(
        bringup_share, "config", "super_lio_mapping_mid360.yaml")

    with_pose_graph = LaunchConfiguration("with_pose_graph")
    with_forwarder = LaunchConfiguration("with_forwarder")
    forwarder_backend = LaunchConfiguration("forwarder_backend")
    with_foxglove = LaunchConfiguration("with_foxglove")
    with_grid_preview = LaunchConfiguration("with_grid_preview")
    with_static_pcd = LaunchConfiguration("with_static_pcd")
    static_pcd_file = LaunchConfiguration("static_pcd_file")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "sensors.launch.py")))

    super_lio = Node(
        package="super_lio",
        executable="super_lio_node",
        name="super_lio_node",
        output="screen",
        parameters=[mapping_lio_yaml],
        arguments=["--ros-args", "--log-level", "info"],
    )

    mapping_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "mapping_backend.launch.py")),
        launch_arguments={
            "with_loop_detection": with_pose_graph,
            "use_legacy_pose_graph": "false",
        }.items(),
    )

    forwarder = Node(
        condition=IfCondition(with_forwarder),
        package="motionslam_pipeline",
        executable="cmd_vel_forwarder",
        name="cmd_vel_forwarder",
        output="screen",
        parameters=[
            pipeline_yaml,
            {"backend": ParameterValue(forwarder_backend, value_type=str)},
        ],
    )

    foxglove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "viz.launch.py")),
        condition=IfCondition(with_foxglove),
    )

    grid_preview = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "gridmap.launch.py")),
        condition=IfCondition(with_grid_preview),
    )

    static_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "map_static.launch.py")),
        launch_arguments={"pcd_file": static_pcd_file}.items(),
        condition=IfCondition(with_static_pcd),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_pose_graph",
            default_value="true",
            description="位姿图后端 (见 mapping_backend.launch.py)",
        ),
        DeclareLaunchArgument(
            "with_forwarder",
            default_value="false",
            description="Nav2/脚本 /cmd_vel → Go2; 试走建议 true",
        ),
        DeclareLaunchArgument(
            "forwarder_backend",
            default_value="obstacles_avoid",
            description="obstacles_avoid (L1 避障, 推荐) | sport",
        ),
        DeclareLaunchArgument(
            "with_foxglove",
            default_value="false",
            description="Foxglove bridge :8765",
        ),
        DeclareLaunchArgument(
            "with_grid_preview",
            default_value="false",
            description="在线 /lio/global_map → /map 预览",
        ),
        DeclareLaunchArgument(
            "with_static_pcd",
            default_value="false",
            description="发布离线 map_viz.pcd; 缺文件时不要开",
        ),
        DeclareLaunchArgument(
            "static_pcd_file",
            default_value=os.path.join(
                os.environ.get("MOTIONSLAM_MAPS", "/ws/maps"), "map_viz.pcd"),
        ),
        sensors,
        super_lio,
        mapping_backend,
        forwarder,
        foxglove,
        grid_preview,
        static_map,
    ])
