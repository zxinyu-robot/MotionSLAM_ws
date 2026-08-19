"""Foxglove 可视化桥

- :8765 foxglove_bridge 3.4+ (foxglove.sdk.v1) — 最新 Foxglove Studio
- :8766 foxglove_bridge 0.8.2 (foxglove.websocket.v1) — 旧客户端 / Lichtblick
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Demo SCAN：主链 pub 精简白名单（两层视图 + BT 事件）
# 调试/大带宽/输入话题见 foxglove_profile:=full
DEMO_SCAN_WHITELIST = [
    "/tf",
    "/tf_static",
    # Super-LIO
    "/lio/pose",
    "/lio/path",
    "/lio/odom",
    "/lio/robo/odom",
    "/lio/imu/odom",
    "/lio/cloud_world",
    "/lio/robot_marker",
    # 局部 SCAN
    "/grid_map/occupancy_inflate",
    "/planning/trajectory_path",
    "/goal_point",
    "/goal_pose",
    "/optimal_list",
    "/a_star_list",
    "/init_list",
    "/initial_path",
    "/planning/bspline",
    # 全局 PCT
    "/tomogram",
    "/global_path",
    "/goal_marker",
    "/way_point",
    "/clicked_point",
    # 语义 BEV (Step 1c)
    "/semantic/bev_grid_world",
    # 编排
    "/demo/mission/event",
]

TOPIC_WHITELIST_FULL = [
    "/lio/.*",
    "/tf",
    "/tf_static",
    "/livox/.*",
    "/map",
    "/map_cloud",
    "/plan",
    "/goal_pose",
    "/move_base_simple/goal",
    "/initial_path",
    "/global_costmap/.*",
    "/local_costmap/.*",
    "/grid_map/.*",
    "/planning/.*",
    "/demo/.*",
    "/navigation_context/.*",
    "/goal_point",
    "/global_list",
    "/optimal_list",
    "/a_star_list",
    "/init_list",
    "/motion/command",
    "/cmd_vel",
    "/lowstate",
    "/lf/lowstate",
    "/motor_monitor/.*",
]


def _setup(context, *args, **kwargs):
    profile = LaunchConfiguration("foxglove_profile").perform(context).strip().lower()
    debug = LaunchConfiguration("debug").perform(context)
    with_legacy = LaunchConfiguration("with_legacy_bridge").perform(context)
    bind_addr = LaunchConfiguration("foxglove_address").perform(context).strip()
    bind_port = int(LaunchConfiguration("foxglove_port").perform(context).strip() or "8765")

    whitelist = DEMO_SCAN_WHITELIST if profile == "demo_scan" else TOPIC_WHITELIST_FULL
    # demo 模式不拉全图参数，避免 edge/rgb 等节点 param 超时刷屏
    capabilities = ["clientPublish"] if profile == "demo_scan" else [
        "clientPublish",
        "parameters",
        "parametersSubscribe",
        "services",
    ]

    bringup_prefix = get_package_prefix("motionslam_bringup")
    legacy_script = os.path.join(
        bringup_prefix, "lib", "motionslam_bringup", "foxglove_bridge_legacy.sh")

    actions = [
        LogInfo(msg=[
            f"Foxglove profile={profile} topics={len(whitelist)} "
            f"ws://{bind_addr}:{bind_port} (wlan, 非雷达端口)",
        ]),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "port": bind_port,
                "address": bind_addr,
                "debug": debug.lower() in ("true", "1", "yes"),
                "topic_whitelist": whitelist,
                "capabilities": capabilities,
            }],
            output="screen",
        ),
    ]
    if with_legacy.lower() in ("true", "1", "yes"):
        actions.append(
            ExecuteProcess(cmd=["bash", legacy_script], output="screen")
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("debug", default_value="false",
                              description="foxglove_bridge 详细 WebSocket 日志"),
        DeclareLaunchArgument("with_legacy_bridge", default_value="false",
                              description="8766 旧协议 bridge"),
        DeclareLaunchArgument(
            "foxglove_profile",
            default_value="demo_scan",
            description="demo_scan=轻量白名单 | full=全量",
        ),
        DeclareLaunchArgument(
            "foxglove_address",
            default_value="${GO2_IP}",
            description="Foxglove 绑定地址（wlan IP，勿用 eth0/雷达网段）",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove WebSocket 端口（8765=sdk.v1，非 Livox UDP 561xx）",
        ),
        OpaqueFunction(function=_setup),
    ])
