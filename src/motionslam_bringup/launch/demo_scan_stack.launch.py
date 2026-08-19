"""Demo 阶段 3D：无先验建图 + SCAN 连续局部 + BT 编排（无 Nav2 实时规划）."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _profile_flags(profile: str) -> dict[str, bool]:
    p = profile.strip().lower()
    if p == "demo_medium":
        return {
            "loop_detection": True,
            "local_map": True,
            "stance_gate": True,
            "use_map_frame": True,
        }
    if p == "demo_reloc":
        return {
            "loop_detection": False,
            "local_map": True,
            "stance_gate": True,
            "use_map_frame": True,
        }
    # demo_short (default)
    return {
        "loop_detection": False,
        "local_map": False,
        "stance_gate": False,
        "use_map_frame": False,
    }


def _setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("motionslam_bringup")
    demo_yaml = os.path.join(bringup_share, "config", "demo_scan_planner.yaml")
    edge_uplink_yaml = os.path.join(bringup_share, "config", "edge_uplink.yaml")
    mapping_yaml = os.path.join(bringup_share, "config", "super_lio_mid360.yaml")
    pipeline_yaml = os.path.join(bringup_share, "config", "pipeline.yaml")
    waypoints_yaml = LaunchConfiguration("waypoints_file").perform(context)
    if not os.path.isabs(waypoints_yaml):
        candidate = os.path.join(bringup_share, "config", waypoints_yaml)
        waypoints_yaml = candidate if os.path.isfile(candidate) else waypoints_yaml

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_bool = use_sim_time.lower() in ("true", "1", "yes")
    autostart_lifecycle = LaunchConfiguration("autostart_lifecycle").perform(context)
    with_forwarder = LaunchConfiguration("with_forwarder").perform(context)
    with_pct_planner = LaunchConfiguration("with_pct_planner").perform(context)
    pct_enabled = with_pct_planner.lower() in ("true", "1", "yes")
    with_semantic_objnav = LaunchConfiguration("with_semantic_objnav").perform(context)
    semantic_objnav_enabled = with_semantic_objnav.lower() in ("true", "1", "yes")
    with_semantic_bev = LaunchConfiguration("with_semantic_bev").perform(context)
    bev_enabled = with_semantic_bev.lower() in ("true", "1", "yes")
    with_pct_python = LaunchConfiguration("with_pct_python").perform(context)
    pct_python = with_pct_python.lower() in ("true", "1", "yes")
    with_glass = LaunchConfiguration("with_glass_aware").perform(context)
    glass_enabled = with_glass.lower() in ("true", "1", "yes")

    profile = LaunchConfiguration("demo_profile").perform(context)
    with_loop_detection_arg = LaunchConfiguration("with_loop_detection").perform(context)
    flags = _profile_flags(profile)

    # 显式 WITH_LOOP_DETECTION 覆盖 profile
    if with_loop_detection_arg.lower() in ("true", "1", "yes"):
        flags["loop_detection"] = True
        flags["local_map"] = True
        flags["stance_gate"] = True
        flags["use_map_frame"] = True
    elif with_loop_detection_arg.lower() in ("false", "0", "no"):
        flags["loop_detection"] = False

    cloud_topic = "/local_map/cloud" if flags["use_map_frame"] else "/lio/cloud_world"
    scan_cloud_topic = "/perception/scan_cloud_fused" if glass_enabled else cloud_topic
    body_pose_topic = "/local_map/body_pose" if flags["use_map_frame"] else "/lio/robo/odom"
    grid_frame = "map" if flags["use_map_frame"] else "world"

    mapping_params = [
        mapping_yaml,
        {
            "lio.map.save_map": profile.lower() == "demo_medium",
            "lio.output.cloud_world_topic.enabled": True,
        },
    ]

    scan_planner_params = [
        demo_yaml,
        {
            "fsm.navi_mode": 3 if pct_enabled else 1,
            "grid_map.frame_id": grid_frame,
            "grid_map.debug_topics": True,
            "grid_map.always_publish_viz": True,
            "grid_map.viz_minimal": True,
            "viz.always_publish": True,
        },
    ]

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "sensors.launch.py")
            )
        ),
        Node(
            package="super_lio",
            executable="super_lio_node",
            name="super_lio_node",
            output="screen",
            parameters=mapping_params + [{"use_sim_time": use_sim_bool}],
        ),
        Node(
            package="motionslam_pipeline",
            executable="demo_navigation_context",
            name="demo_navigation_context",
            output="screen",
            parameters=[
                {
                    "session_id": LaunchConfiguration("session_id").perform(context),
                    "tile_id": LaunchConfiguration("tile_id").perform(context),
                    "map_id": "demo_live_map",
                }
            ],
        ),
    ]

    if flags["loop_detection"]:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "mapping_backend.launch.py")
                ),
                launch_arguments={
                    "with_loop_detection": "true",
                    "use_legacy_pose_graph": "false",
                }.items(),
            )
        )

    if flags["stance_gate"]:
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="stance_gate_node.py",
                name="stance_gate_node",
                output="screen",
                parameters=[pipeline_yaml],
            )
        )

    if flags["local_map"]:
        actions.append(
            Node(
                package="motionslam_pipeline",
                executable="local_map_node",
                name="local_map_node",
                output="screen",
                parameters=[pipeline_yaml],
            )
        )

    if glass_enabled:
        actions.append(
            Node(
                package="motionslam_pipeline",
                executable="virtual_obstacle_merge",
                name="virtual_obstacle_merge",
                output="screen",
                parameters=[
                    {
                        "lidar_topic": cloud_topic,
                        "virtual_obstacle_topic": "/perception/virtual_obstacles",
                        "output_topic": "/perception/scan_cloud_fused",
                    }
                ],
            )
        )

    scan_stack_nodes = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=scan_planner_params + [{"use_sim_time": use_sim_bool}],
            remappings=[
                ("body_pose", body_pose_topic),
                ("sensor_pose", body_pose_topic),
                ("cloud", scan_cloud_topic),
                ("initial_path", "/initial_path"),
            ],
        ),
        Node(
            condition=IfCondition(with_forwarder),
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[demo_yaml, {"use_sim_time": use_sim_bool}],
            remappings=[
                ("body_pose", body_pose_topic),
            ],
        ),
        Node(
            condition=IfCondition(with_forwarder),
            package="motionslam_pipeline",
            executable="cmd_vel_forwarder",
            name="cmd_vel_forwarder",
            output="screen",
            parameters=[pipeline_yaml, demo_yaml],
        ),
    ]
    bt_params = {
        "waypoints_file": waypoints_yaml,
        "autostart_lifecycle": autostart_lifecycle.lower() in ("true", "1", "yes"),
        "demo_profile": profile,
        "default_frame_id": grid_frame,
        "glass_aware_enabled": glass_enabled,
        "return_home_enabled": True,
    }
    if semantic_objnav_enabled:
        bt_params.update(
            {
                "mission_mode": "semantic_objnav",
                "waypoints_file": "",
            }
        )
    if pct_enabled:
        bt_params["pct_global_nav_enabled"] = True
        bt_params["scan_track_pct_path"] = True
        bt_params["pct_fallback_direct"] = False
    scan_stack_nodes.append(
        Node(
            package="motionslam_bringup",
            executable="demo_scan_bt_orchestrator.py",
            name="demo_scan_bt_orchestrator",
            output="screen",
            parameters=[
                demo_yaml,
                bt_params,
            ],
        )
    )
    actions.extend(scan_stack_nodes)

    if pct_enabled:
        actions.append(
            Node(
                package="motionslam_pipeline",
                executable="pct_resident_planner",
                name="pct_planner",
                output="screen",
                remappings=[
                    ("explored_areas", cloud_topic),
                    ("state_estimation", body_pose_topic),
                ],
            )
        )
        actions.append(
            LogInfo(
                msg=(
                    "PCT C++ 常驻规划：预热 tomogram + /global_path；"
                    f"SCAN fsm.navi_mode={3 if pct_enabled else 1}；"
                    "SCAN 跟踪 BT /initial_path；断网原路返航不依赖 A*"
                )
            )
        )
        if pct_python:
            try:
                pct_share = get_package_share_directory("pct_planner")
            except Exception as exc:
                raise RuntimeError(
                    "with_pct_python:=true 需要先编译 PCT：./scripts/dev/build_pct_planner.sh"
                ) from exc
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pct_share, "launch", "pct_planner.launch.py")
                    ),
                    launch_arguments={
                        "local_mode": "false",
                        "cloud_topic": cloud_topic,
                        "odom_topic": body_pose_topic,
                    }.items(),
                )
            )

    foxglove = LaunchConfiguration("with_foxglove").perform(context)
    motor_monitor = LaunchConfiguration("with_motor_monitor").perform(context)
    with_edge_uplink = LaunchConfiguration("with_edge_uplink").perform(context)
    session_id = LaunchConfiguration("session_id").perform(context)
    if glass_enabled:
        pipeline_yaml_path = os.path.join(bringup_share, "config", "pipeline.yaml")
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="glass_suspect_layer.py",
                name="glass_suspect_layer",
                output="screen",
                parameters=[
                    pipeline_yaml_path,
                    {
                        "cloud_topic": cloud_topic,
                        "odom_topic": body_pose_topic,
                        "obstacle_frame_id": grid_frame,
                    },
                ],
            )
        )
    if with_edge_uplink.lower() in ("true", "1", "yes"):
        tile_id_val = LaunchConfiguration("tile_id").perform(context)
        floor_id_val = LaunchConfiguration("floor_id").perform(context)
        edge_host_val = LaunchConfiguration("edge_host").perform(context)
        uplink_identity = {
            "session_id": session_id,
            "tile_id": tile_id_val,
            "floor_id": floor_id_val,
            "map_id": "demo_live_map",
            "frame_id": grid_frame,
            "edge_host": edge_host_val,
        }
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="data_layer_registry_node.py",
                name="data_layer_registry",
                output="screen",
                parameters=[edge_uplink_yaml, uplink_identity],
            )
        )
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="subgraph_publisher_node.py",
                name="subgraph_publisher",
                output="screen",
                parameters=[
                    edge_uplink_yaml,
                    uplink_identity,
                ],
            )
        )
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="rgb_keyframe_uplink_node.py",
                name="rgb_keyframe_uplink",
                output="screen",
                parameters=[
                    edge_uplink_yaml,
                    uplink_identity,
                ],
            )
        )
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="execution_feedback_node.py",
                name="execution_feedback",
                output="screen",
                parameters=[edge_uplink_yaml, uplink_identity],
            )
        )
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="directive_receiver_node.py",
                name="directive_receiver",
                output="screen",
                parameters=[edge_uplink_yaml, uplink_identity],
            )
        )
        if semantic_objnav_enabled:
            actions.append(
                Node(
                    package="motionslam_bringup",
                    executable="command_executor_node.py",
                    name="command_executor",
                    output="screen",
                    parameters=[edge_uplink_yaml, uplink_identity],
                )
            )
            actions.append(
                LogInfo(
                    msg=(
                        "MVPI1 semantic_objnav：command_executor + BT semantic 模式；"
                        "mock: python3 scripts/offline/mock_semantic_directive.py "
                        "--kind action_group --nav-x 2.0 --nav-y 0.0"
                    )
                )
            )
    if bev_enabled:
        actions.append(
            Node(
                package="motionslam_pipeline",
                executable="bev_glass_layer",
                name="bev_glass_layer",
                output="screen",
                parameters=[
                    {
                        "engine_path": "",
                        "bev_grid_topic": "/semantic/bev_grid_world",
                        "virtual_obstacle_topic": "/perception/virtual_obstacles_bev",
                    }
                ],
            )
        )
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="semantic_token_uplink_node.py",
                name="semantic_token_uplink",
                output="screen",
                parameters=[
                    edge_uplink_yaml,
                    {
                        "session_id": session_id,
                        "tile_id": LaunchConfiguration("tile_id").perform(context),
                        "floor_id": LaunchConfiguration("floor_id").perform(context),
                        "map_id": "demo_live_map",
                        "edge_host": LaunchConfiguration("edge_host").perform(context),
                    },
                ],
            )
        )
        actions.append(
            LogInfo(
                msg=(
                    "BEV 玻璃层：TensorRT C++ bev_glass_layer → "
                    "/perception/virtual_obstacles_bev（并入 SCAN；无 engine 不做 stub 推理）"
                )
            )
        )
    if motor_monitor.lower() in ("true", "1", "yes"):
        pipeline_yaml = os.path.join(bringup_share, "config", "pipeline.yaml")
        actions.append(
            Node(
                package="motionslam_bringup",
                executable="motor_monitor_node.py",
                name="motor_monitor_node",
                output="screen",
                parameters=[pipeline_yaml],
            )
        )
    if foxglove.lower() in ("true", "1", "yes"):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "viz.launch.py")
                ),
                launch_arguments={
                    "foxglove_profile": "demo_scan",
                    "with_legacy_bridge": "false",
                    "foxglove_address": LaunchConfiguration("foxglove_bind").perform(context),
                    "foxglove_port": LaunchConfiguration("foxglove_port").perform(context),
                }.items(),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "robot_viz.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        )
        ws_root = os.path.abspath(os.path.join(bringup_share, "..", "..", "..", ".."))
        planning_viz_script = os.path.join(
            ws_root, "src", "motionslam_bringup", "scripts", "planning_trajectory_viz.py"
        )
        if not os.path.isfile(planning_viz_script):
            from ament_index_python.packages import get_package_prefix

            planning_viz_script = os.path.join(
                get_package_prefix("motionslam_bringup"),
                "lib",
                "motionslam_bringup",
                "planning_trajectory_viz.py",
            )
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    planning_viz_script,
                    "--ros-args",
                    "-p",
                    f"frame_id:={grid_frame}",
                    "-p",
                    "bspline_topic:=/planning/bspline",
                ],
                output="screen",
            )
        )

    delay = float(LaunchConfiguration("lifecycle_autostart_delay_sec").perform(context))
    if autostart_lifecycle.lower() in ("true", "1", "yes") and delay > 0:
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    Node(
                        package="motionslam_bringup",
                        executable="demo_scan_lifecycle_autostart.py",
                        name="demo_scan_lifecycle_autostart_once",
                        output="screen",
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("motionslam_bringup")
    default_waypoints = os.path.join(
        bringup_share, "config", "demo_scan_waypoints.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "with_foxglove",
                default_value="false",
                description="Foxglove bridge（Step 1 lean 默认关）",
            ),
            DeclareLaunchArgument(
                "foxglove_bind",
                default_value="${GO2_IP}",
                description="Foxglove 绑定地址（wlan IP）",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="Foxglove WebSocket 端口",
            ),
            DeclareLaunchArgument(
                "with_motor_monitor",
                default_value="false",
                description="订阅 lowstate 并发布 /motor_monitor/* + 异常日志",
            ),
            DeclareLaunchArgument(
                "with_glass_aware",
                default_value="true",
                description="启用 glass_suspect_layer + BT GlassAwareRecovery + 虚拟障碍并入 SCAN",
            ),
            DeclareLaunchArgument(
                "with_edge_uplink",
                default_value="false",
                description="启用 v1 边侧链路：子图:9877 + RGB:9878 + directive listen:9879",
            ),
            DeclareLaunchArgument(
                "with_semantic_objnav",
                default_value="false",
                description="MVPI1：启用 command_executor（需 with_edge_uplink:=true）",
            ),
            DeclareLaunchArgument(
                "with_semantic_bev",
                default_value="false",
                description="Step 1c：TensorRT BEV 玻璃层（bev_glass_layer）；不做 Python stub 推理",
            ),
            DeclareLaunchArgument(
                "demo_profile",
                default_value="demo_short",
                description="demo_short | demo_medium | demo_reloc",
            ),
            DeclareLaunchArgument(
                "with_loop_detection",
                default_value="auto",
                description="auto=跟 profile；true/false 强制覆盖",
            ),
            DeclareLaunchArgument("session_id", default_value="demo_session"),
            DeclareLaunchArgument("tile_id", default_value="main"),
            DeclareLaunchArgument("floor_id", default_value="floor_01"),
            DeclareLaunchArgument(
                "edge_host",
                default_value="${EDGE_HOST}",
                description="边侧 uplink 目标 IP（9877/9878/9880）",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=default_waypoints,
                description="Demo subgoal 序列 YAML",
            ),
            DeclareLaunchArgument(
                "autostart_lifecycle",
                default_value="true",
                description="启动后自动 configure+activate+开始任务",
            ),
            DeclareLaunchArgument(
                "with_forwarder",
                default_value="true",
                description="false=干跑：不启 forwarder/closed_loop，狗不 BalanceStand",
            ),
            DeclareLaunchArgument(
                "with_pct_planner",
                default_value="true",
                description="开机常驻 PCT C++ 规划器（预热 tomogram + /global_path）",
            ),
            DeclareLaunchArgument(
                "with_pct_python",
                default_value="false",
                description="额外启动 Python/Open3D PCT（会与 C++ /build_tomogram 冲突，仅调试）",
            ),
            DeclareLaunchArgument(
                "lifecycle_autostart_delay_sec",
                default_value="0.0",
                description="延迟 autostart（秒）",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
