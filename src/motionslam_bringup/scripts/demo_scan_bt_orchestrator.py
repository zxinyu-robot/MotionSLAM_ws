#!/usr/bin/env python3
"""Demo BT 编排：门控 LIO/点云 → 逐个 subgoal 下发 → 规划失败早停."""
from __future__ import annotations

import json
import math
import os
import sys
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Deque, Optional

import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from scan_planner_msgs.msg import Bspline
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from plan_fail_recovery import (  # noqa: E402
    DriftAssessment,
    DriftThresholds,
    RecoveryBranch,
    assess_drift,
    should_pause_replan,
    yaw_delta,
)
from return_home import (  # noqa: E402
    densify_xy,
    record_breadcrumb,
    reverse_xy_for_scan,
    should_return_home,
    uplink_is_lost,
)
from semantic_mission import SemanticSubgoal, heading_to_goal, subgoal_from_pose  # noqa: E402
from frontier_scorer import FrontierScore, select_best_frontier  # noqa: E402
from subgraph_snapshot import FrontierView  # noqa: E402


class MissionMode(str, Enum):
    DEMO_WAYPOINTS = "demo_waypoints"
    SEMANTIC_OBJNAV = "semantic_objnav"


class SyncParameterClient:
    """Humble rclpy 无 AsyncParametersClient，用 set_parameters 服务同步改参."""

    def __init__(self, node: Node, remote_node_name: str) -> None:
        self._node = node
        self._client = node.create_client(
            SetParameters, f"/{remote_node_name}/set_parameters"
        )

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    @staticmethod
    def _to_ros_parameter(param: Parameter) -> RosParameter:
        ros_param = RosParameter()
        ros_param.name = param.name
        value = ParameterValue()
        if param.type_ == Parameter.Type.DOUBLE:
            value.type = ParameterType.PARAMETER_DOUBLE
            value.double_value = float(param.value)
        elif param.type_ == Parameter.Type.BOOL:
            value.type = ParameterType.PARAMETER_BOOL
            value.bool_value = bool(param.value)
        elif param.type_ == Parameter.Type.INTEGER:
            value.type = ParameterType.PARAMETER_INTEGER
            value.integer_value = int(param.value)
        elif param.type_ == Parameter.Type.STRING:
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = str(param.value)
        else:
            value.type = ParameterType.PARAMETER_NOT_SET
        ros_param.value = value
        return ros_param

    def set_parameters(self, params: list[Parameter]) -> None:
        req = SetParameters.Request()
        req.parameters = [self._to_ros_parameter(p) for p in params]
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
        if future.result() is not None and not future.result().results:
            return
        if future.result() is not None:
            for res in future.result().results:
                if not res.successful:
                    raise RuntimeError(res.reason)


class BtStatus(str, Enum):
    IDLE = "idle"
    WAIT_CLOUD = "wait_cloud"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


class ExecPhase(str, Enum):
    IDLE = "idle"
    DISPATCH = "dispatch"
    EXECUTE = "execute"
    SEARCH = "search"
    RECOVERY_WAIT = "recovery_wait"
    GLASS_HALT = "glass_halt"
    GLASS_BACKUP = "glass_backup"
    PLAN_FAIL_HOLD = "plan_fail_hold"
    DRIFT_ICP = "drift_icp"
    ESCAPE = "escape"
    RETURN_HOME = "return_home"


class PgoState(str, Enum):
    IDLE = "IDLE"
    OPTIMIZING = "OPTIMIZING"
    APPLIED = "APPLIED"
    UNKNOWN = "UNKNOWN"


@dataclass
class Subgoal:
    x: float
    y: float
    z: float
    frame_id: str = "world"
    command_id: str = ""
    language_query: str = ""
    frontier_id: str = ""


def _semantic_to_subgoal(item: SemanticSubgoal) -> Subgoal:
    return Subgoal(
        x=item.x,
        y=item.y,
        z=item.z,
        frame_id=item.frame_id,
        command_id=item.command_id,
        language_query=item.language_query,
    )


@dataclass
class MissionState:
    mission_id: str = ""
    status: BtStatus = BtStatus.IDLE
    phase: ExecPhase = ExecPhase.IDLE
    current_index: int = 0
    waypoints: list[Subgoal] = field(default_factory=list)
    started_at: float = 0.0
    last_progress_at: float = 0.0
    last_pose_xy: tuple[float, float] = (0.0, 0.0)
    replan_fail_count: int = 0
    stuck_redispatch_count: int = 0
    best_dist_to_subgoal: float = float("inf")
    initial_dist_to_subgoal: float = float("inf")
    execute_entered_at: float = 0.0
    goal_dispatched_at: float = 0.0
    goal_dispatch_time: Optional[TimeMsg] = None
    last_bspline_at: float = 0.0
    ready_traj_id: int = -1
    detail: str = ""
    recovery_step: int = 0
    recovery_until: float = 0.0
    sidestep_sign: float = 1.0
    glass_recovery_count: int = 0
    plan_fail_recovery_count: int = 0
    escape_step: int = 0
    escape_start_xy: tuple[float, float] = (0.0, 0.0)
    dispatch_anchor_xy: tuple[float, float] = (0.0, 0.0)
    dispatch_anchor_yaw: float = 0.0
    icp_started_at: float = 0.0
    plan_fail_reason: str = ""


def load_waypoints(path: str, body_height: float) -> list[Subgoal]:
    if not path or not os.path.isfile(path):
        return []
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    block = data.get("demo_scan_waypoints") or data
    params = block.get("ros__parameters") or block
    height = float(params.get("body_height", body_height))
    raw = params.get("waypoints") or []
    out: list[Subgoal] = []
    for item in raw:
        if isinstance(item, dict):
            out.append(
                Subgoal(
                    float(item["x"]),
                    float(item["y"]),
                    float(item.get("z", height)),
                )
            )
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            out.append(
                Subgoal(
                    float(item[0]),
                    float(item[1]),
                    float(item[2] if len(item) > 2 else height),
                )
            )
    return out


class DemoScanBtOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("demo_scan_bt_orchestrator")
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("subgoal_reach_m", 0.45)
        self.declare_parameter("min_execute_before_reach_s", 1.5)
        self.declare_parameter("progress_dist_delta_m", 0.12)
        self.declare_parameter("stuck_timeout_s", 60.0)
        self.declare_parameter("stuck_redispatch_max", 3)
        self.declare_parameter("replan_fail_threshold", 8)
        self.declare_parameter("plan_ready_timeout_s", 8.0)
        self.declare_parameter("cloud_ready_min_messages", 10)
        self.declare_parameter("cloud_ready_window_s", 4.0)
        self.declare_parameter("cloud_settle_s", 3.0)
        self.declare_parameter("max_subgoal_dist_m", 8.0)
        self.declare_parameter("autostart_lifecycle", True)
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("body_height", 0.35)
        self.declare_parameter("goal_topic", "move_base_simple/goal")
        self.declare_parameter("demo_profile", "demo_short")
        self.declare_parameter("recovery_wait_s", 2.5)
        self.declare_parameter("sidestep_m", 0.30)
        self.declare_parameter("glass_aware_enabled", True)
        self.declare_parameter("glass_slow_factor", 0.30)
        self.declare_parameter("glass_halt_wait_s", 1.5)
        self.declare_parameter("glass_detour_timeout_s", 5.0)
        self.declare_parameter("glass_max_recovery", 3)
        self.declare_parameter("glass_backup_m", 0.20)
        self.declare_parameter("require_stance_on_glass_halt", True)
        self.declare_parameter("plan_fail_recovery_enabled", True)
        self.declare_parameter("plan_fail_pause_after", 2)
        self.declare_parameter("max_plan_fail_recovery", 4)
        self.declare_parameter("drift_trans_m_threshold", 0.25)
        self.declare_parameter("drift_yaw_deg_threshold", 8.0)
        self.declare_parameter("drift_odom_jump_m", 0.45)
        self.declare_parameter("drift_anchor_trans_m", 0.35)
        self.declare_parameter("drift_anchor_yaw_deg", 10.0)
        self.declare_parameter("icp_wait_timeout_s", 20.0)
        self.declare_parameter("escape_backup_m", 0.80)
        self.declare_parameter("escape_sidestep_m", 0.35)
        self.declare_parameter("escape_step_timeout_s", 8.0)
        self.declare_parameter("relocation_max_fitness", 0.35)
        self.declare_parameter("mission_mode", "demo_waypoints")
        self.declare_parameter("default_frame_id", "world")
        self.declare_parameter("semantic_goal_topic", "/demo/mission/semantic_goal")
        self.declare_parameter("semantic_autostart_mission", False)
        self.declare_parameter("frontier_snapshot_topic", "/demo/mission/frontier_snapshot")
        self.declare_parameter("search_max_time_s", 180.0)
        self.declare_parameter("search_max_distance_m", 30.0)
        self.declare_parameter("search_max_frontier_visits", 12)
        self.declare_parameter("pct_global_nav_enabled", False)
        self.declare_parameter("scan_track_pct_path", False)
        self.declare_parameter("pct_initial_path_topic", "/initial_path")
        self.declare_parameter("pct_goal_topic", "/goal_pose")
        self.declare_parameter("pct_global_path_topic", "/global_path")
        self.declare_parameter("pct_fallback_direct", False)
        self.declare_parameter("return_home_enabled", True)
        self.declare_parameter("breadcrumb_spacing_m", 0.60)
        self.declare_parameter("breadcrumb_max_poses", 400)
        self.declare_parameter("link_lost_timeout_s", 8.0)
        self.declare_parameter("return_home_arrive_m", 0.80)
        self.declare_parameter("uplink_ok_topic", "/demo/mission/uplink_ok")
        self.declare_parameter("return_home_topic", "/demo/mission/return_home")

        self._tick_hz = float(self.get_parameter("tick_hz").value)
        self._reach_m = float(self.get_parameter("subgoal_reach_m").value)
        self._min_execute_before_reach = float(
            self.get_parameter("min_execute_before_reach_s").value
        )
        self._progress_dist_delta = float(self.get_parameter("progress_dist_delta_m").value)
        self._stuck_timeout = float(self.get_parameter("stuck_timeout_s").value)
        self._stuck_redispatch_max = int(self.get_parameter("stuck_redispatch_max").value)
        self._replan_fail_threshold = int(self.get_parameter("replan_fail_threshold").value)
        self._plan_ready_timeout = float(self.get_parameter("plan_ready_timeout_s").value)
        self._cloud_min_msgs = int(self.get_parameter("cloud_ready_min_messages").value)
        self._cloud_window_s = float(self.get_parameter("cloud_ready_window_s").value)
        self._cloud_settle_s = float(self.get_parameter("cloud_settle_s").value)
        self._max_subgoal_dist = float(self.get_parameter("max_subgoal_dist_m").value)
        self._autostart = bool(self.get_parameter("autostart_lifecycle").value)
        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._demo_profile = str(self.get_parameter("demo_profile").value)
        self._recovery_wait_s = float(self.get_parameter("recovery_wait_s").value)
        self._sidestep_m = float(self.get_parameter("sidestep_m").value)
        self._glass_enabled = bool(self.get_parameter("glass_aware_enabled").value)
        self._glass_slow_factor = float(self.get_parameter("glass_slow_factor").value)
        self._glass_halt_wait = float(self.get_parameter("glass_halt_wait_s").value)
        self._glass_detour_timeout = float(self.get_parameter("glass_detour_timeout_s").value)
        self._glass_max_recovery = int(self.get_parameter("glass_max_recovery").value)
        self._glass_backup_m = float(self.get_parameter("glass_backup_m").value)
        self._require_stance_glass = bool(
            self.get_parameter("require_stance_on_glass_halt").value
        )
        self._plan_fail_recovery_enabled = bool(
            self.get_parameter("plan_fail_recovery_enabled").value
        )
        self._plan_fail_pause_after = int(self.get_parameter("plan_fail_pause_after").value)
        self._max_plan_fail_recovery = int(self.get_parameter("max_plan_fail_recovery").value)
        self._drift_thresholds = DriftThresholds(
            trans_m=float(self.get_parameter("drift_trans_m_threshold").value),
            yaw_deg=float(self.get_parameter("drift_yaw_deg_threshold").value),
            odom_jump_m=float(self.get_parameter("drift_odom_jump_m").value),
            anchor_trans_m=float(self.get_parameter("drift_anchor_trans_m").value),
            anchor_yaw_deg=float(self.get_parameter("drift_anchor_yaw_deg").value),
        )
        self._icp_wait_timeout = float(self.get_parameter("icp_wait_timeout_s").value)
        self._escape_backup_m = float(self.get_parameter("escape_backup_m").value)
        self._escape_sidestep_m = float(self.get_parameter("escape_sidestep_m").value)
        self._escape_step_timeout = float(self.get_parameter("escape_step_timeout_s").value)
        self._relocation_max_fitness = float(
            self.get_parameter("relocation_max_fitness").value
        )
        mode_raw = str(self.get_parameter("mission_mode").value).strip().lower()
        try:
            self._mission_mode = MissionMode(mode_raw)
        except ValueError:
            self._mission_mode = MissionMode.DEMO_WAYPOINTS
            self.get_logger().warn(f"Unknown mission_mode={mode_raw}, use demo_waypoints")
        self._default_frame_id = str(self.get_parameter("default_frame_id").value)
        self._semantic_goal_topic = str(self.get_parameter("semantic_goal_topic").value)
        self._semantic_autostart = bool(self.get_parameter("semantic_autostart_mission").value)
        wp_file = str(self.get_parameter("waypoints_file").value)
        body_h = float(self.get_parameter("body_height").value)

        if self._mission_mode == MissionMode.SEMANTIC_OBJNAV:
            self._mission = MissionState(waypoints=[])
        else:
            self._mission = MissionState(waypoints=load_waypoints(wp_file, body_h))
        self._odom: Optional[Odometry] = None
        self._lifecycle = "unconfigured"
        self._nav_ready = False
        self._cancel_requested = False
        self._pending_autostart = False
        self._cloud_timestamps: Deque[float] = deque()
        self._last_traj_id_seen = -1
        self._traj_id_at_dispatch = -1
        self._cloud_ready_since: Optional[float] = None
        self._pgo_state = PgoState.UNKNOWN
        self._last_pgo_state = PgoState.UNKNOWN
        self._glass_suspect = False
        self._glass_confirmed = False
        self._glass_state = "NONE"
        self._glass_laser_no_return = False
        self._glass_near_obstacle = False
        self._stance_ok = True
        self._slow_mode_active = False
        self._vel_defaults = {"scan_max_vel": 0.5, "cl_max_vx": 0.5, "cl_max_vy": 0.35}
        self._drift_estimate_raw: Optional[str] = None
        self._relocation_fitness: Optional[float] = None
        self._relocation_converged = False
        self._prev_odom_xy: Optional[tuple[float, float]] = None
        self._pending_emergency_hold = False
        self._pending_semantic_start = False
        self._semantic_command_id = ""
        self._semantic_language_query = ""
        self._semantic_task_kind = "nav"
        self._semantic_directive: Optional[dict[str, Any]] = None
        self._semantic_directive_valid = False
        self._frontier_snapshot: list[FrontierView] = []
        self._visited_frontier_ids: set[str] = set()
        self._search_started_at = 0.0
        self._search_origin_xy: tuple[float, float] = (0.0, 0.0)
        self._search_frontier_visits = 0
        self._search_max_time_s = float(self.get_parameter("search_max_time_s").value)
        self._search_max_distance_m = float(self.get_parameter("search_max_distance_m").value)
        self._search_max_frontier_visits = int(self.get_parameter("search_max_frontier_visits").value)
        self._pct_global_nav_enabled = bool(self.get_parameter("pct_global_nav_enabled").value)
        self._scan_track_pct_path = bool(self.get_parameter("scan_track_pct_path").value)
        self._pct_fallback_direct = bool(self.get_parameter("pct_fallback_direct").value)
        self._pct_tomogram_built = False
        self._pct_ref_path: Optional[Path] = None
        self._pct_request_stamp_ns: Optional[int] = None
        self._return_home_enabled = bool(self.get_parameter("return_home_enabled").value)
        self._breadcrumb_spacing = float(self.get_parameter("breadcrumb_spacing_m").value)
        self._breadcrumb_max = int(self.get_parameter("breadcrumb_max_poses").value)
        self._link_lost_timeout = float(self.get_parameter("link_lost_timeout_s").value)
        self._return_home_arrive_m = float(self.get_parameter("return_home_arrive_m").value)
        self._breadcrumbs: list[tuple[float, float]] = []
        self._return_home_end_xy: Optional[tuple[float, float]] = None
        self._uplink_ever_ok = False
        self._uplink_last_ok_mono = 0.0
        self._uplink_ok: Optional[bool] = None
        self._return_home_pending: Optional[str] = None

        self._scan_param_client = SyncParameterClient(self, "scan_planner_node")
        self._cl_param_client = SyncParameterClient(self, "closed_loop_controller")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._lifecycle_pub = self.create_publisher(String, "/demo/lifecycle/transition", 10)
        self._event_pub = self.create_publisher(String, "/demo/mission/event", 10)
        self._status_pub = self.create_publisher(String, "/demo/mission/status", qos)
        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._pct_goal_pub = None
        self._pct_initial_path_pub = None
        self._pct_build_client = None
        init_topic = str(self.get_parameter("pct_initial_path_topic").value)
        self._pct_initial_path_pub = self.create_publisher(Path, init_topic, 10)
        if self._pct_global_nav_enabled:
            pct_goal_topic = str(self.get_parameter("pct_goal_topic").value)
            self._pct_goal_pub = self.create_publisher(PoseStamped, pct_goal_topic, 10)
            path_topic = str(self.get_parameter("pct_global_path_topic").value)
            path_qos = QoSProfile(depth=1)
            path_qos.reliability = ReliabilityPolicy.RELIABLE
            path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(Path, path_topic, self._on_pct_global_path, path_qos)
            self._pct_build_client = self.create_client(Trigger, "/build_tomogram")

        self.create_subscription(Odometry, "/lio/robo/odom", self._on_odom, 10)
        self.create_subscription(PointCloud2, "/lio/cloud_world", self._on_cloud, 10)
        self.create_subscription(Bspline, "/planning/bspline", self._on_bspline, 10)
        self.create_subscription(String, "/demo/lifecycle/state", self._on_lifecycle, qos)
        self.create_subscription(Bool, "navigation_context/ready", self._on_ready, qos)
        self.create_subscription(String, "/demo/mission/cancel", self._on_cancel, 10)
        self.create_subscription(String, "/demo/mission/start", self._on_start, 10)
        if self._mission_mode == MissionMode.SEMANTIC_OBJNAV:
            self.create_subscription(
                PoseStamped, self._semantic_goal_topic, self._on_semantic_goal, 10
            )
            self.create_subscription(String, "/demo/mission/event", self._on_external_mission_event, 20)
            self.create_subscription(String, "/semantic/directive", self._on_semantic_directive, 10)
            self.create_subscription(Bool, "/semantic/valid", self._on_semantic_valid, 10)
            frontier_topic = str(self.get_parameter("frontier_snapshot_topic").value)
            self.create_subscription(String, frontier_topic, self._on_frontier_snapshot, 10)
        self.create_subscription(String, "/lio/backend/pgo_state", self._on_pgo_state, 10)
        self.create_subscription(String, "/lio/backend/drift_estimate", self._on_drift_estimate, 10)
        self.create_subscription(Bool, "/lio/relocation/converged", self._on_relocation_converged, 10)
        self.create_subscription(Float64, "/lio/relocation/fitness", self._on_relocation_fitness, 10)
        if self._glass_enabled:
            self.create_subscription(
                Bool, "/perception/glass_suspect/flag", self._on_glass_flag, 10
            )
            self.create_subscription(
                String, "/perception/glass_suspect/summary", self._on_glass_summary, 10
            )
        self.create_subscription(Bool, "/local_map/stance_ok", self._on_stance_ok, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter("uplink_ok_topic").value),
            self._on_uplink_ok,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("return_home_topic").value),
            self._on_return_home_cmd,
            10,
        )

        period = max(0.05, 1.0 / self._tick_hz)
        self._timer = self.create_timer(period, self._tick)

        if self._autostart:
            self._send_lifecycle("configure")
            self._send_lifecycle("activate")
            if self._mission_mode == MissionMode.DEMO_WAYPOINTS:
                self._pending_autostart = True

        self.get_logger().info(
            f"Demo BT: mode={self._mission_mode.value}, "
            f"{len(self._mission.waypoints)} subgoals, "
            f"cloud_gate>={self._cloud_min_msgs}@{self._cloud_window_s}s, "
            f"plan_timeout={self._plan_ready_timeout}s, goal={self._goal_topic}, "
            f"profile={self._demo_profile}, glass={self._glass_enabled}, "
            f"plan_fail_recovery={self._plan_fail_recovery_enabled}, "
            f"return_home={self._return_home_enabled}"
        )

    def _on_glass_flag(self, msg: Bool) -> None:
        self._glass_suspect = bool(msg.data)

    def _on_glass_summary(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._glass_laser_no_return = bool(data.get("laser_no_return"))
            self._glass_near_obstacle = bool(data.get("near_obstacle"))
            self._glass_suspect = bool(data.get("glass_suspect", self._glass_suspect))
            self._glass_confirmed = bool(data.get("glass_confirmed", self._glass_confirmed))
            self._glass_state = str(data.get("state", self._glass_state))
        except json.JSONDecodeError:
            pass

    def _is_glass_solid(self) -> bool:
        return self._glass_state in ("CONFIRMED", "SOLID") or self._glass_confirmed

    def _on_stance_ok(self, msg: Bool) -> None:
        self._stance_ok = bool(msg.data)

    def _on_drift_estimate(self, msg: String) -> None:
        self._drift_estimate_raw = msg.data

    def _on_relocation_converged(self, msg: Bool) -> None:
        self._relocation_converged = bool(msg.data)

    def _on_relocation_fitness(self, msg: Float64) -> None:
        self._relocation_fitness = float(msg.data)

    def _yaw_from_odom(self) -> float:
        if self._odom is None:
            return 0.0
        q = self._odom.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _odom_jump_m(self) -> float:
        xy = self._pose_xy()
        if xy is None or self._prev_odom_xy is None:
            return 0.0
        return math.hypot(xy[0] - self._prev_odom_xy[0], xy[1] - self._prev_odom_xy[1])

    def _anchor_drift(self) -> tuple[float, float]:
        xy = self._pose_xy()
        if xy is None:
            return 0.0, 0.0
        trans = math.hypot(
            xy[0] - self._mission.dispatch_anchor_xy[0],
            xy[1] - self._mission.dispatch_anchor_xy[1],
        )
        yaw_deg = math.degrees(
            yaw_delta(self._mission.dispatch_anchor_yaw, self._yaw_from_odom())
        )
        return trans, yaw_deg

    def _assess_drift_now(self) -> DriftAssessment:
        anchor_trans, anchor_yaw = self._anchor_drift()
        return assess_drift(
            drift_estimate_raw=self._drift_estimate_raw,
            odom_jump_m=self._odom_jump_m(),
            anchor_trans_m=anchor_trans,
            anchor_yaw_deg=anchor_yaw,
            relocation_fitness=self._relocation_fitness,
            relocation_max_fitness=self._relocation_max_fitness,
            thresholds=self._drift_thresholds,
        )

    def _enter_plan_fail_hold(self, reason: str) -> None:
        xy = self._pose_xy()
        if xy is not None:
            self._mission.dispatch_anchor_xy = xy
            self._mission.dispatch_anchor_yaw = self._yaw_from_odom()
        self._mission.plan_fail_reason = reason
        self._mission.phase = ExecPhase.PLAN_FAIL_HOLD
        self._mission.recovery_until = time.time()
        self.get_logger().warn(f"PlanFailHold: 停规划 → 评估漂移 ({reason})")
        self._emit("plan_fail_hold", reason=reason)

    def _publish_initialpose_for_icp(self) -> None:
        if self._odom is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self._odom.pose.pose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self._initialpose_pub.publish(msg)
        self.get_logger().info("DriftICP: 已发 /initialpose 触发重定位")

    def _dispatch_escape_goal(self, mode: str) -> None:
        xy = self._pose_xy()
        if xy is None or self._odom is None:
            return
        yaw = self._yaw_from_odom()
        bz = max(self._odom.pose.pose.position.z, 0.3)
        if mode == "backup":
            tx = xy[0] - self._escape_backup_m * math.cos(yaw)
            ty = xy[1] - self._escape_backup_m * math.sin(yaw)
        else:
            tx = xy[0] + self._escape_sidestep_m * (-math.sin(yaw))
            ty = xy[1] + self._escape_sidestep_m * math.cos(yaw)
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = tx
        goal.pose.position.y = ty
        goal.pose.position.z = bz
        goal.pose.orientation = self._odom.pose.pose.orientation
        self._goal_pub.publish(goal)
        self._mission.goal_dispatched_at = time.time()
        self._mission.goal_dispatch_time = self.get_clock().now().to_msg()
        self._traj_id_at_dispatch = self._last_traj_id_seen
        self._mission.last_progress_at = time.time()
        self.get_logger().warn(f"Escape {mode} → ({tx:.2f}, {ty:.2f})")
        self._emit("escape_step", mode=mode, x=round(tx, 3), y=round(ty, 3))

    def _finish_plan_fail_recovery(self) -> None:
        self._mission.plan_fail_recovery_count += 1
        self._mission.replan_fail_count = 0
        self._mission.stuck_redispatch_count = 0
        self._mission.recovery_step = 0
        self._mission.escape_step = 0
        self._mission.last_progress_at = time.time()
        self._pending_emergency_hold = False
        self._mission.phase = ExecPhase.DISPATCH
        self._emit(
            "plan_fail_recovery_done",
            recovery_count=self._mission.plan_fail_recovery_count,
        )
        self._dispatch_current_subgoal()

    def _maybe_enter_plan_fail_recovery(self, reason: str) -> bool:
        if not self._plan_fail_recovery_enabled:
            return False
        if self._mission.plan_fail_recovery_count >= self._max_plan_fail_recovery:
            return False
        self._enter_plan_fail_hold(reason)
        return True

    def _is_glass_conservative(self) -> bool:
        if not self._glass_enabled:
            return False
        if self._is_glass_solid():
            return True
        return self._glass_suspect or (
            self._glass_laser_no_return and self._glass_near_obstacle
        )

    def _apply_slow_mode(self, enable: bool) -> None:
        if enable == self._slow_mode_active:
            return
        factor = self._glass_slow_factor if enable else 1.0
        scan_vel = self._vel_defaults["scan_max_vel"] * factor
        cl_vx = self._vel_defaults["cl_max_vx"] * factor
        cl_vy = self._vel_defaults["cl_max_vy"] * factor
        params = [
            Parameter("manager.max_vel", Parameter.Type.DOUBLE, scan_vel),
            Parameter("optimization.max_vel", Parameter.Type.DOUBLE, scan_vel),
        ]
        cl_params = [
            Parameter("max_vx", Parameter.Type.DOUBLE, cl_vx),
            Parameter("max_vy", Parameter.Type.DOUBLE, cl_vy),
        ]
        try:
            if self._scan_param_client.wait_for_service(timeout_sec=0.5):
                self._scan_param_client.set_parameters(params)
            if self._cl_param_client.wait_for_service(timeout_sec=0.5):
                self._cl_param_client.set_parameters(cl_params)
            self._slow_mode_active = enable
            self.get_logger().info(
                f"SlowApproach {'ON' if enable else 'OFF'} "
                f"(factor={factor:.2f}, scan_vel={scan_vel:.2f})"
            )
            self._emit("glass_slow_mode", enabled=enable, factor=round(factor, 3))
        except Exception as exc:
            self.get_logger().warn(f"SlowApproach 参数更新失败: {exc}")

    def _dispatch_backup_goal(self) -> None:
        xy = self._pose_xy()
        if xy is None or self._odom is None:
            return
        q = self._odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        bx = xy[0] - self._glass_backup_m * math.cos(yaw)
        by = xy[1] - self._glass_backup_m * math.sin(yaw)
        bz = max(self._odom.pose.pose.position.z, 0.3)
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = bx
        goal.pose.position.y = by
        goal.pose.position.z = bz
        goal.pose.orientation = q
        self._goal_pub.publish(goal)
        self._mission.phase = ExecPhase.GLASS_BACKUP
        self._mission.goal_dispatched_at = time.time()
        self._mission.last_progress_at = time.time()
        self.get_logger().warn(f"Glass BackUp {self._glass_backup_m}m → ({bx:.2f}, {by:.2f})")
        self._emit(
            "glass_backup",
            subgoal_index=self._mission.current_index,
            x=round(bx, 3),
            y=round(by, 3),
        )

    def _on_pgo_state(self, msg: String) -> None:
        raw = msg.data.strip().upper()
        try:
            self._pgo_state = PgoState(raw)
        except ValueError:
            self._pgo_state = PgoState.UNKNOWN

        if (
            self._last_pgo_state in (PgoState.OPTIMIZING, PgoState.UNKNOWN)
            and self._pgo_state == PgoState.APPLIED
            and self._mission.status == BtStatus.RUNNING
            and self._mission.phase in (ExecPhase.EXECUTE, ExecPhase.DISPATCH)
        ):
            self.get_logger().info("PGO APPLIED → 无感 subgoal 纠偏 redispatch")
            self._emit("pgo_applied_redispatch", subgoal_index=self._mission.current_index)
            self._mission.last_progress_at = time.time()
            self._dispatch_current_subgoal()
        self._last_pgo_state = self._pgo_state

    def _send_lifecycle(self, transition: str) -> None:
        msg = String()
        msg.data = transition
        self._lifecycle_pub.publish(msg)
        self.get_logger().info(f"lifecycle transition → {transition}")

    def _emit(self, event: str, **payload: Any) -> None:
        body = {
            "event": event,
            "mission_id": self._mission.mission_id,
            "status": self._mission.status.value,
            "phase": self._mission.phase.value,
            "subgoal_index": self._mission.current_index,
            "subgoal_total": len(self._mission.waypoints),
            "lifecycle": self._lifecycle,
            "navigation_ready": self._nav_ready,
            "replan_fail_count": self._mission.replan_fail_count,
            **payload,
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._event_pub.publish(msg)
        status = String()
        status.data = json.dumps(body, ensure_ascii=False)
        self._status_pub.publish(status)

    def _on_odom(self, msg: Odometry) -> None:
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._odom = msg
        if self._prev_odom_xy is not None:
            jump = math.hypot(xy[0] - self._prev_odom_xy[0], xy[1] - self._prev_odom_xy[1])
            if jump > self._drift_thresholds.odom_jump_m and self._mission.phase in (
                ExecPhase.DISPATCH,
                ExecPhase.EXECUTE,
            ):
                self.get_logger().warn(f"Odom jump {jump:.2f}m detected")
        self._prev_odom_xy = xy
        if (
            self._return_home_enabled
            and self._mission.phase != ExecPhase.RETURN_HOME
            and self._mission.status == BtStatus.RUNNING
        ):
            record_breadcrumb(
                self._breadcrumbs,
                xy[0],
                xy[1],
                spacing_m=self._breadcrumb_spacing,
                max_poses=self._breadcrumb_max,
            )

    def _on_cloud(self, _msg: PointCloud2) -> None:
        now = time.time()
        self._cloud_timestamps.append(now)
        cutoff = now - self._cloud_window_s
        while self._cloud_timestamps and self._cloud_timestamps[0] < cutoff:
            self._cloud_timestamps.popleft()

    @staticmethod
    def _is_emergency_stop_bspline(msg: Bspline) -> bool:
        if len(msg.pos_pts) < 2:
            return True
        first = msg.pos_pts[0]
        return all(
            abs(p.x - first.x) < 1e-3
            and abs(p.y - first.y) < 1e-3
            and abs(p.z - first.z) < 1e-3
            for p in msg.pos_pts
        )

    @staticmethod
    def _bspline_start_time(msg: Bspline) -> rclpy.time.Time:
        return rclpy.time.Time.from_msg(msg.start_time)

    def _on_bspline(self, msg: Bspline) -> None:
        traj_id = int(msg.traj_id)
        self._last_traj_id_seen = max(self._last_traj_id_seen, traj_id)
        if self._mission.phase != ExecPhase.DISPATCH:
            return
        if self._mission.goal_dispatch_time is None:
            return
        if self._is_emergency_stop_bspline(msg):
            if self._scan_track_pct_path:
                self.get_logger().warn("Emergency bspline ignored (scan_track_pct_path)")
                return
            if self._plan_fail_recovery_enabled:
                self._pending_emergency_hold = True
                self.get_logger().warn("Emergency bspline → 待进入 PlanFailHold")
            return
        if traj_id <= self._traj_id_at_dispatch:
            return
        if self._bspline_start_time(msg) < rclpy.time.Time.from_msg(self._mission.goal_dispatch_time):
            return
        self._mission.last_bspline_at = time.time()
        self._mission.ready_traj_id = traj_id

    def _on_lifecycle(self, msg: String) -> None:
        self._lifecycle = msg.data.strip()

    def _on_ready(self, msg: Bool) -> None:
        self._nav_ready = bool(msg.data)

    def _on_cancel(self, _msg: String) -> None:
        self._cancel_requested = True

    def _on_semantic_directive(self, msg: String) -> None:
        try:
            self._semantic_directive = json.loads(msg.data)
            self._semantic_directive_valid = True
        except json.JSONDecodeError:
            self._semantic_directive_valid = False

    def _on_semantic_valid(self, msg: Bool) -> None:
        self._semantic_directive_valid = bool(msg.data)
        if not self._semantic_directive_valid:
            self._semantic_directive = None

    def _on_frontier_snapshot(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        raw = body.get("frontiers") or []
        out: list[FrontierView] = []
        for item in raw:
            if not isinstance(item, dict) or not item.get("frontier_id"):
                continue
            try:
                out.append(
                    FrontierView(
                        frontier_id=str(item["frontier_id"]),
                        x=float(item["x"]),
                        y=float(item["y"]),
                        z=float(item.get("z", 0.35)),
                        geom_score=float(item.get("geom_score", 1.0)),
                    )
                )
            except (TypeError, ValueError):
                continue
        self._frontier_snapshot = out

    def _active_directive(self) -> Optional[dict[str, Any]]:
        if self._semantic_directive_valid and self._semantic_directive:
            return self._semantic_directive
        return self._semantic_directive

    def _search_budget_exhausted(self) -> bool:
        if self._search_started_at <= 0:
            return False
        now = time.time()
        if now - self._search_started_at > self._search_max_time_s:
            return True
        if self._search_frontier_visits >= self._search_max_frontier_visits:
            return True
        xy = self._pose_xy()
        if xy is None:
            return False
        dist = math.hypot(xy[0] - self._search_origin_xy[0], xy[1] - self._search_origin_xy[1])
        return dist > self._search_max_distance_m

    def _finish_search_exhausted(self, reason: str = "budget") -> None:
        self._mission.status = BtStatus.FAILED
        self._mission.phase = ExecPhase.IDLE
        self._semantic_task_kind = "nav"
        self._apply_slow_mode(False)
        self._emit(
            "search_exhausted",
            command_id=self._semantic_command_id,
            language_query=self._semantic_language_query,
            reason=reason,
            visits=self._search_frontier_visits,
        )
        self._emit("mission_failed", detail="search_exhausted", reason=reason)
        self._send_lifecycle("deactivate")

    def _arm_search_mission(self) -> None:
        if self._mission.status in (BtStatus.WAIT_CLOUD, BtStatus.RUNNING):
            self.get_logger().warn(
                f"Search mission ignored: mission status={self._mission.status.value}"
            )
            return
        self._semantic_task_kind = "search"
        self._visited_frontier_ids = set()
        self._search_frontier_visits = 0
        self._search_started_at = 0.0
        self._mission.waypoints = []
        self._mission.current_index = 0
        self.get_logger().info(
            f"SemanticObjNav SEARCH armed command_id={self._semantic_command_id}"
        )
        self._emit(
            "search_mission_armed",
            command_id=self._semantic_command_id,
            language_query=self._semantic_language_query,
        )
        if self._lifecycle == "active" and self._nav_ready:
            self._begin_mission_wait_cloud()
        else:
            self._pending_semantic_start = True

    def _dispatch_search_frontier(self) -> None:
        if self._search_budget_exhausted():
            self._finish_search_exhausted("budget")
            return
        directive = self._active_directive()
        picked: Optional[FrontierScore] = select_best_frontier(
            self._frontier_snapshot,
            directive,
            exclude_ids=self._visited_frontier_ids,
        )
        if picked is None:
            self._finish_search_exhausted("no_frontier")
            return
        sg = Subgoal(
            x=picked.x,
            y=picked.y,
            z=picked.z,
            frame_id=self._default_frame_id,
            command_id=self._semantic_command_id,
            language_query=self._semantic_language_query,
            frontier_id=picked.frontier_id,
        )
        self._mission.waypoints = [sg]
        self._mission.current_index = 0
        self._mission.phase = ExecPhase.SEARCH
        self._emit(
            "frontier_selected",
            frontier_id=picked.frontier_id,
            total_score=round(picked.total_score, 4),
            geom_score=round(picked.geom_score, 4),
            semantic_score=round(picked.semantic_score, 4),
            x=round(picked.x, 3),
            y=round(picked.y, 3),
        )
        self._emit(
            "search_explore",
            command_id=self._semantic_command_id,
            frontier_id=picked.frontier_id,
        )
        self._dispatch_current_subgoal()

    def _on_search_frontier_reached(self, offset_m: float) -> None:
        wp = self._mission.waypoints[0] if self._mission.waypoints else None
        frontier_id = wp.frontier_id if wp else ""
        if frontier_id:
            self._visited_frontier_ids.add(frontier_id)
        self._search_frontier_visits += 1
        self._mission.replan_fail_count = 0
        self._mission.stuck_redispatch_count = 0
        self._mission.recovery_step = 0
        self._mission.best_dist_to_subgoal = float("inf")
        self._apply_slow_mode(False)
        self._emit(
            "search_frontier_reached",
            frontier_id=frontier_id,
            offset_m=round(offset_m, 3),
            visits=self._search_frontier_visits,
        )
        if self._search_budget_exhausted():
            self._finish_search_exhausted("budget")
            return
        self._dispatch_search_frontier()

    def _on_external_mission_event(self, msg: String) -> None:
        if self._mission_mode != MissionMode.SEMANTIC_OBJNAV:
            return
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = str(body.get("event", ""))
        if event == "task_accepted":
            self._semantic_command_id = str(body.get("command_id", ""))
            self._semantic_language_query = str(body.get("language_query", ""))
            stage_ids = [str(x) for x in body.get("stage_ids") or []]
            if "SEARCH_SEMANTIC_TARGET" in stage_ids and "NAV_TO_DESTINATION" not in stage_ids:
                self._arm_search_mission()
            elif "NAV_TO_DESTINATION" in stage_ids and self._semantic_task_kind == "search":
                self._stop_search_for_nav_handoff()
                self._semantic_task_kind = "nav"
                self._pending_semantic_start = True
        elif event == "task_rejected":
            self._pending_semantic_start = False
            if self._mission.status in (BtStatus.WAIT_CLOUD, BtStatus.RUNNING):
                self._cancel_mission()

    def _on_semantic_goal(self, msg: PoseStamped) -> None:
        if self._mission_mode != MissionMode.SEMANTIC_OBJNAV:
            return
        body_h = float(self.get_parameter("body_height").value)
        sg = subgoal_from_pose(msg, body_h)
        sg.command_id = self._semantic_command_id
        sg.language_query = self._semantic_language_query
        self._arm_semantic_mission(sg)

    def _arm_semantic_mission(self, sg: SemanticSubgoal) -> None:
        if self._mission.status in (BtStatus.WAIT_CLOUD, BtStatus.RUNNING):
            self.get_logger().warn(
                f"Semantic goal ignored: mission status={self._mission.status.value}"
            )
            return
        self._semantic_task_kind = "nav"
        self._mission.waypoints = [_semantic_to_subgoal(sg)]
        self._mission.current_index = 0
        self.get_logger().info(
            f"SemanticObjNav armed command_id={sg.command_id} "
            f"→ ({sg.x:.2f}, {sg.y:.2f}, {sg.z:.2f}) frame={sg.frame_id}"
        )
        self._emit(
            "semantic_goal_armed",
            command_id=sg.command_id,
            language_query=sg.language_query,
            x=round(sg.x, 3),
            y=round(sg.y, 3),
            z=round(sg.z, 3),
            frame_id=sg.frame_id,
        )
        if self._lifecycle == "active" and self._nav_ready:
            self._begin_mission_wait_cloud()
        else:
            self._pending_semantic_start = True

    def _on_start(self, _msg: String) -> None:
        if self._lifecycle != "active":
            self._send_lifecycle("configure")
            self._send_lifecycle("activate")
        if self._mission_mode == MissionMode.SEMANTIC_OBJNAV and not self._mission.waypoints:
            if self._semantic_task_kind != "search":
                self._pending_semantic_start = True
                self.get_logger().info("SemanticObjNav: 等待 /demo/mission/semantic_goal")
                return
        self._begin_mission_wait_cloud()

    def _cloud_ready(self) -> bool:
        return len(self._cloud_timestamps) >= self._cloud_min_msgs

    def _begin_mission_wait_cloud(self) -> None:
        if not self._mission.waypoints:
            if self._mission_mode == MissionMode.SEMANTIC_OBJNAV:
                if self._semantic_task_kind == "search":
                    pass
                else:
                    self.get_logger().warn("SemanticObjNav: 无 subgoal，等待 semantic_goal")
                    return
            else:
                self._mission.status = BtStatus.FAILED
                self._emit("mission_rejected", detail="no_waypoints")
                return
        self._mission.mission_id = uuid.uuid4().hex[:12]
        self._mission.status = BtStatus.WAIT_CLOUD
        self._mission.phase = ExecPhase.IDLE
        self._mission.current_index = 0
        self._mission.replan_fail_count = 0
        self._mission.stuck_redispatch_count = 0
        self._mission.glass_recovery_count = 0
        self._mission.plan_fail_recovery_count = 0
        self._mission.escape_step = 0
        self._pending_emergency_hold = False
        self._mission.best_dist_to_subgoal = float("inf")
        self._mission.started_at = time.time()
        self._mission.last_progress_at = self._mission.started_at
        self._cancel_requested = False
        self._cloud_ready_since = None
        self._emit("mission_wait_cloud")

    def _start_mission(self) -> None:
        self._mission.status = BtStatus.RUNNING
        if self._semantic_task_kind == "search":
            xy = self._pose_xy()
            self._search_origin_xy = xy if xy is not None else (0.0, 0.0)
            self._search_started_at = time.time()
            self._mission.phase = ExecPhase.SEARCH
            self._emit("mission_started", mission_mode="semantic_search")
            self._emit(
                "search_explore",
                command_id=self._semantic_command_id,
                language_query=self._semantic_language_query,
            )
            self._dispatch_search_frontier()
            return
        self._mission.phase = ExecPhase.DISPATCH
        detail = "semantic_objnav" if self._mission_mode == MissionMode.SEMANTIC_OBJNAV else "demo_waypoints"
        self._emit("mission_started", mission_mode=detail)
        if self._mission_mode == MissionMode.SEMANTIC_OBJNAV:
            wp = self._mission.waypoints[0] if self._mission.waypoints else None
            self._emit(
                "search_navigate",
                command_id=wp.command_id if wp else "",
                language_query=wp.language_query if wp else "",
            )
        self._dispatch_current_subgoal()

    def _pose_xy(self) -> Optional[tuple[float, float]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        return p.x, p.y

    def _dist_to_subgoal(self, index: int) -> Optional[float]:
        xy = self._pose_xy()
        if xy is None or index >= len(self._mission.waypoints):
            return None
        wp = self._mission.waypoints[index]
        return math.hypot(xy[0] - wp.x, xy[1] - wp.y)

    def _ensure_pct_tomogram(self) -> None:
        if self._pct_tomogram_built or self._pct_build_client is None:
            return
        if not self._pct_build_client.service_is_ready():
            self.get_logger().warn("/build_tomogram 服务未就绪")
            return
        fut = self._pct_build_client.call_async(Trigger.Request())

        def _done(_fut) -> None:
            res = _fut.result()
            if res is not None and res.success:
                self._pct_tomogram_built = True
                self.get_logger().info(f"/build_tomogram: {res.message}")
            else:
                self.get_logger().warn(
                    f"/build_tomogram failed: {None if res is None else res.message}"
                )

        fut.add_done_callback(_done)

    def _copy_path_for_scan(self, msg: Path) -> Path:
        raw: list[tuple[float, float]] = []
        xy = self._pose_xy()
        if xy is not None and msg.poses:
            hx = msg.poses[0].pose.position.x
            hy = msg.poses[0].pose.position.y
            if math.hypot(xy[0] - hx, xy[1] - hy) > 0.6:
                raw.append((float(xy[0]), float(xy[1])))
        for ps in msg.poses:
            raw.append((float(ps.pose.position.x), float(ps.pose.position.y)))
        return self._xy_to_path(densify_xy(raw, step_m=0.20), tag="pct")

    @staticmethod
    def _stamp_ns(stamp: TimeMsg) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _xy_to_path(self, xy_list: list[tuple[float, float]], *, tag: str) -> Path:
        path = Path()
        path.header.frame_id = self._default_frame_id or "world"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in xy_list:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.get_logger().info(f"SCAN initial_path poses={len(path.poses)} ({tag})")
        return path

    def _publish_pct_initial_path(self) -> None:
        if self._pct_initial_path_pub is None or self._pct_ref_path is None:
            return
        path = self._pct_ref_path
        path.header.stamp = self.get_clock().now().to_msg()
        self._pct_initial_path_pub.publish(path)

    def _on_uplink_ok(self, msg: Bool) -> None:
        self._uplink_ok = bool(msg.data)
        if self._uplink_ok:
            self._uplink_ever_ok = True
            self._uplink_last_ok_mono = time.monotonic()

    def _on_return_home_cmd(self, _msg: String) -> None:
        self._return_home_pending = "explicit"
        if self._mission.status != BtStatus.RUNNING:
            self._start_return_home("explicit")


    def _publish_xy_initial_path(
        self, xy_list: list[tuple[float, float]], *, tag: str = "return_home"
    ) -> None:
        if self._pct_initial_path_pub is None or len(xy_list) < 2:
            return
        path = self._xy_to_path(densify_xy(xy_list, step_m=0.20), tag=tag)
        self._pct_ref_path = path
        self._pct_initial_path_pub.publish(path)

    def _publish_direct_line_to_subgoal(self) -> None:
        xy = self._pose_xy()
        idx = self._mission.current_index
        if xy is None or idx >= len(self._mission.waypoints):
            return
        wp = self._mission.waypoints[idx]
        self._publish_xy_initial_path([(xy[0], xy[1]), (wp.x, wp.y)], tag="pct_fallback")

    def _start_return_home(self, reason: str) -> bool:
        if not self._return_home_enabled:
            return False
        if self._mission.phase == ExecPhase.RETURN_HOME:
            return True
        xy = self._pose_xy()
        if xy is None:
            return False
        rev = reverse_xy_for_scan(self._breadcrumbs, xy)
        if len(rev) < 2:
            self.get_logger().warn(f"return_home skipped ({reason}): trail too short")
            return False
        self._publish_xy_initial_path(rev)
        self._return_home_end_xy = rev[-1]
        self._mission.status = BtStatus.RUNNING
        self._mission.phase = ExecPhase.RETURN_HOME
        self._mission.last_progress_at = time.time()
        self._mission.last_pose_xy = xy
        self._return_home_pending = None
        self._apply_slow_mode(True)
        self.get_logger().warn(
            f"RETURN_HOME reason={reason} poses={len(rev)} end=({rev[-1][0]:.2f},{rev[-1][1]:.2f})"
        )
        self._emit("return_home_started", reason=reason, poses=len(rev))
        return True

    def _maybe_trigger_return_home(self) -> bool:
        lost = uplink_is_lost(
            ever_ok=self._uplink_ever_ok,
            last_ok_mono=self._uplink_last_ok_mono,
            now_mono=time.monotonic(),
            timeout_s=self._link_lost_timeout,
            uplink_ok=self._uplink_ok,
        )
        reason = should_return_home(
            explicit=self._return_home_pending == "explicit",
            glass_trap=self._return_home_pending == "glass_trap",
            link_lost=lost,
            already_returning=self._mission.phase == ExecPhase.RETURN_HOME,
            trail_len=len(self._breadcrumbs),
        )
        if reason is None:
            return False
        return self._start_return_home(reason)

    def _on_pct_global_path(self, msg: Path) -> None:
        response_stamp_ns = self._stamp_ns(msg.header.stamp)
        if (
            self._pct_request_stamp_ns is None
            or response_stamp_ns != self._pct_request_stamp_ns
        ):
            self.get_logger().warn(
                "Ignore stale PCT path: "
                f"response={response_stamp_ns} expected={self._pct_request_stamp_ns}"
            )
            return
        if not msg.poses:
            self._emit(
                "pct_path_failed",
                subgoal_index=self._mission.current_index,
                reason="empty_path",
            )
            if self._scan_track_pct_path and self._pct_fallback_direct:
                self.get_logger().warn("PCT empty path; explicit direct fallback enabled")
                self._publish_direct_line_to_subgoal()
            else:
                self.get_logger().warn("PCT empty path; wait for BT recovery")
                self._mission.goal_dispatched_at = 0.0
            return
        if len(msg.poses) < 2:
            self.get_logger().warn("PCT path has fewer than two poses; reject")
            self._mission.goal_dispatched_at = 0.0
            return
        path_length = 0.0
        for previous, current in zip(msg.poses, msg.poses[1:]):
            path_length += math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
        distance_to_goal = self._dist_to_subgoal(self._mission.current_index)
        if (
            path_length < 0.2
            and distance_to_goal is not None
            and distance_to_goal > self._reach_m
        ):
            self.get_logger().warn(
                f"Reject PCT path length={path_length:.2f}m while "
                f"goal remains {distance_to_goal:.2f}m away"
            )
            self._mission.goal_dispatched_at = 0.0
            return
        self.get_logger().info(f"PCT /global_path poses={len(msg.poses)}")
        if self._scan_track_pct_path:
            self._pct_ref_path = self._copy_path_for_scan(msg)
            self._publish_pct_initial_path()
            self._emit(
                "pct_path_received",
                subgoal_index=self._mission.current_index,
                input_poses=len(msg.poses),
                scan_poses=len(self._pct_ref_path.poses),
            )

    def _dispatch_current_subgoal(self, *, force_glass_detour: bool = False) -> None:
        idx = self._mission.current_index
        if idx >= len(self._mission.waypoints):
            return
        if self._is_glass_conservative():
            self._apply_slow_mode(True)
        wp = self._mission.waypoints[idx]
        dist = self._dist_to_subgoal(idx)
        if dist is not None and dist > self._max_subgoal_dist:
            self._fail_mission(
                "subgoal_too_far",
                subgoal_index=idx,
                distance_m=round(dist, 2),
                max_m=self._max_subgoal_dist,
            )
            return

        target_x, target_y, target_z = wp.x, wp.y, wp.z
        retry = self._mission.stuck_redispatch_count
        xy = self._pose_xy()
        if force_glass_detour and xy is not None:
            self._mission.recovery_step = max(self._mission.recovery_step, 2)
        if (
            retry > 0
            and not self._pct_global_nav_enabled
            and xy is not None
            and dist is not None
            and dist > self._reach_m
        ):
            scale = max(0.35, 1.0 - 0.25 * retry)
            target_x = xy[0] + (wp.x - xy[0]) * scale
            target_y = xy[1] + (wp.y - xy[1]) * scale
            if self._mission.recovery_step >= 2:
                target_x = xy[0] - self._sidestep_m * self._mission.sidestep_sign
                target_y = xy[1] + (wp.y - xy[1]) * scale
                self._mission.sidestep_sign *= -1.0
            elif retry >= 3 and abs(wp.x - xy[0]) < 0.15:
                target_x = xy[0] + 0.5
                target_y = xy[1] + (wp.y - xy[1]) * scale
            self.get_logger().warn(
                f"Subgoal retry scale={scale:.2f} step={self._mission.recovery_step} → "
                f"({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
            )

        goal = PoseStamped()
        goal.header.frame_id = wp.frame_id or self._default_frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target_x
        goal.pose.position.y = target_y
        goal.pose.position.z = target_z
        if xy is not None:
            qz, qw = heading_to_goal(xy, (target_x, target_y))
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        else:
            goal.pose.orientation.w = 1.0
        if self._pct_global_nav_enabled and self._pct_goal_pub is not None:
            self._ensure_pct_tomogram()
            self._pct_ref_path = None
            self._pct_request_stamp_ns = self._stamp_ns(goal.header.stamp)
            self._pct_goal_pub.publish(goal)
            if not self._scan_track_pct_path:
                self._goal_pub.publish(goal)
        else:
            self._pct_request_stamp_ns = None
            self._goal_pub.publish(goal)
        if self._scan_track_pct_path and self._pct_fallback_direct:
            self._publish_direct_line_to_subgoal()
        xy = self._pose_xy()
        if xy is not None:
            self._mission.dispatch_anchor_xy = xy
            self._mission.dispatch_anchor_yaw = self._yaw_from_odom()
        self._mission.phase = ExecPhase.DISPATCH
        self._mission.goal_dispatched_at = time.time()
        self._mission.goal_dispatch_time = self.get_clock().now().to_msg()
        self._traj_id_at_dispatch = self._last_traj_id_seen
        self._mission.last_bspline_at = 0.0
        self._mission.ready_traj_id = -1
        self._mission.execute_entered_at = 0.0
        self._mission.best_dist_to_subgoal = dist if dist is not None else float("inf")
        self.get_logger().info(
            f"AdvanceSubgoal {idx + 1}/{len(self._mission.waypoints)} "
            f"→ ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
        )
        self._emit(
            "subgoal_dispatched",
            subgoal_index=idx,
            x=target_x,
            y=target_y,
            z=target_z,
            retry=retry,
        )

    def _fail_mission(self, detail: str, **payload: Any) -> None:
        self._mission.status = BtStatus.FAILED
        self._mission.phase = ExecPhase.IDLE
        self._apply_slow_mode(False)
        self._emit("mission_failed", detail=detail, **payload)
        self._send_lifecycle("deactivate")

    def _cancel_mission(self) -> None:
        self._mission.status = BtStatus.CANCELED
        self._mission.phase = ExecPhase.IDLE
        self._apply_slow_mode(False)
        self._emit("mission_canceled")
        self._send_lifecycle("deactivate")

    def _stop_search_for_nav_handoff(self) -> None:
        """G4：SEARCH→NAV 切换时软停探索，保持 lifecycle active 等待 semantic_goal."""
        if self._mission.status not in (BtStatus.WAIT_CLOUD, BtStatus.RUNNING):
            return
        self._mission.status = BtStatus.IDLE
        self._mission.phase = ExecPhase.IDLE
        self._mission.waypoints = []
        self._mission.current_index = 0
        self._apply_slow_mode(False)
        self._emit(
            "search_handoff",
            command_id=self._semantic_command_id,
            reason="nav_directive",
        )

    def _tick(self) -> None:
        if self._pending_autostart and self._nav_ready and self._lifecycle == "active":
            self._begin_mission_wait_cloud()
            self._pending_autostart = False

        if (
            self._pending_semantic_start
            and self._nav_ready
            and self._lifecycle == "active"
            and (self._mission.waypoints or self._semantic_task_kind == "search")
            and self._mission.status == BtStatus.IDLE
        ):
            self._begin_mission_wait_cloud()
            self._pending_semantic_start = False

        if self._cancel_requested and self._mission.status in (
            BtStatus.WAIT_CLOUD,
            BtStatus.RUNNING,
        ):
            self._cancel_mission()
            self._cancel_requested = False
            return

        if self._mission.status == BtStatus.WAIT_CLOUD:
            if not self._nav_ready or self._lifecycle != "active":
                return
            if self._odom is None:
                return
            if not self._cloud_ready():
                self._cloud_ready_since = None
                return
            if self._cloud_ready_since is None:
                self._cloud_ready_since = time.time()
                self.get_logger().info(
                    f"Cloud gate passed ({len(self._cloud_timestamps)} msgs), "
                    f"settling {self._cloud_settle_s:.1f}s..."
                )
                return
            if time.time() - self._cloud_ready_since < self._cloud_settle_s:
                return
            self.get_logger().info(
                f"LocalizationReady: cloud msgs={len(self._cloud_timestamps)} "
                f"(>{self._cloud_min_msgs} in {self._cloud_window_s}s)"
            )
            self._emit("cloud_ready", cloud_msgs=len(self._cloud_timestamps))
            self._start_mission()
            return

        if self._mission.status != BtStatus.RUNNING:
            return

        now = time.time()
        if self._maybe_trigger_return_home():
            pass

        if self._mission.phase == ExecPhase.RETURN_HOME:
            xy = self._pose_xy()
            if xy is None or self._return_home_end_xy is None:
                return
            moved = math.hypot(
                xy[0] - self._mission.last_pose_xy[0], xy[1] - self._mission.last_pose_xy[1]
            )
            if moved > 0.15:
                self._mission.last_progress_at = now
                self._mission.last_pose_xy = xy
            dist = math.hypot(
                xy[0] - self._return_home_end_xy[0], xy[1] - self._return_home_end_xy[1]
            )
            if dist <= self._return_home_arrive_m:
                self._mission.status = BtStatus.SUCCEEDED
                self._mission.phase = ExecPhase.IDLE
                self._apply_slow_mode(False)
                self._emit("return_home_done", distance_m=round(dist, 2))
                self._send_lifecycle("deactivate")
            elif now - self._mission.last_progress_at > max(20.0, self._stuck_timeout):
                self._fail_mission("return_home_stuck", distance_m=round(dist, 2))
            return

        if not self._nav_ready or self._lifecycle != "active" or self._odom is None:
            return

        if self._mission.phase == ExecPhase.RECOVERY_WAIT:
            if now < self._mission.recovery_until:
                return
            self.get_logger().info("Recovery Wait 结束 → redispatch")
            self._mission.phase = ExecPhase.DISPATCH
            self._emit("recovery_wait_done", subgoal_index=self._mission.current_index)
            self._dispatch_current_subgoal()
            return

        if self._mission.phase == ExecPhase.GLASS_HALT:
            if now < self._mission.recovery_until:
                if self._require_stance_glass and not self._stance_ok:
                    return
                return
            self.get_logger().info("Glass HaltAndWait 结束 → TryDetour")
            self._mission.glass_recovery_count += 1
            self._mission.phase = ExecPhase.DISPATCH
            self._emit(
                "glass_halt_done",
                subgoal_index=self._mission.current_index,
                glass_recovery=self._mission.glass_recovery_count,
                glass_confirmed=self._glass_confirmed,
                glass_state=self._glass_state,
            )
            self._dispatch_current_subgoal(force_glass_detour=not self._is_glass_solid())
            return

        if self._mission.phase == ExecPhase.GLASS_BACKUP:
            if now - self._mission.goal_dispatched_at > 4.0:
                self._return_home_pending = "glass_trap"
                if self._start_return_home("glass_trap"):
                    return
                self._fail_mission(
                    "glass_trap",
                    subgoal_index=self._mission.current_index,
                    glass_recovery=self._mission.glass_recovery_count,
                    assist_requested=False,
                )
            return

        if self._mission.phase == ExecPhase.PLAN_FAIL_HOLD:
            assessment = self._assess_drift_now()
            self._emit("plan_fail_assess", **assessment.to_payload())
            if assessment.drifted:
                self._mission.phase = ExecPhase.DRIFT_ICP
                self._mission.icp_started_at = now
                if self._demo_profile == "demo_reloc":
                    self._publish_initialpose_for_icp()
                self.get_logger().warn(
                    f"漂移确认 ({assessment.reason}) → DriftICP"
                )
                self._emit("drift_icp_start", **assessment.to_payload())
            else:
                self._mission.phase = ExecPhase.ESCAPE
                self._mission.escape_step = 0
                xy = self._pose_xy()
                if xy is not None:
                    self._mission.escape_start_xy = xy
                self._dispatch_escape_goal("backup")
                self.get_logger().warn("无漂移 → Escape(backup+sidestep)")
                self._emit("escape_start", reason=assessment.reason)
            return

        if self._mission.phase == ExecPhase.DRIFT_ICP:
            icp_done = False
            if self._demo_profile == "demo_reloc" and self._relocation_converged:
                icp_done = True
            elif self._pgo_state == PgoState.APPLIED and self._last_pgo_state in (
                PgoState.OPTIMIZING,
                PgoState.UNKNOWN,
            ):
                icp_done = True
            elif now - self._mission.icp_started_at > self._icp_wait_timeout:
                icp_done = True
                self.get_logger().warn("DriftICP 超时 → 仍尝试再规划")
            if icp_done:
                self._emit("drift_icp_done", waited_s=round(now - self._mission.icp_started_at, 1))
                self._finish_plan_fail_recovery()
            return

        if self._mission.phase == ExecPhase.ESCAPE:
            xy = self._pose_xy()
            assert xy is not None
            moved = math.hypot(xy[0] - self._mission.escape_start_xy[0], xy[1] - self._mission.escape_start_xy[1])
            step_done = (
                moved > self._progress_dist_delta
                or now - self._mission.goal_dispatched_at > self._escape_step_timeout
            )
            if not step_done:
                return
            if self._mission.escape_step == 0:
                self._mission.escape_step = 1
                self._mission.escape_start_xy = xy
                self._dispatch_escape_goal("sidestep")
                return
            self._emit("escape_done", moved_m=round(moved, 2))
            self._finish_plan_fail_recovery()
            return

        if self._pgo_state == PgoState.OPTIMIZING and self._mission.phase not in (
            ExecPhase.DRIFT_ICP,
        ):
            return

        xy = self._pose_xy()
        assert xy is not None
        moved = math.hypot(xy[0] - self._mission.last_pose_xy[0], xy[1] - self._mission.last_pose_xy[1])
        if moved > 0.15:
            self._mission.last_progress_at = time.time()
            self._mission.last_pose_xy = xy

        if self._mission.phase == ExecPhase.DISPATCH:
            if self._pending_emergency_hold:
                if self._maybe_enter_plan_fail_recovery("emergency_bspline"):
                    return
            if self._mission.ready_traj_id > self._traj_id_at_dispatch:
                self._mission.phase = ExecPhase.EXECUTE
                self._mission.execute_entered_at = now
                self._mission.replan_fail_count = 0
                self._mission.stuck_redispatch_count = 0
                dist = self._dist_to_subgoal(self._mission.current_index)
                self._mission.best_dist_to_subgoal = dist if dist is not None else float("inf")
                self._mission.initial_dist_to_subgoal = self._mission.best_dist_to_subgoal
                if self._is_glass_conservative():
                    self._apply_slow_mode(True)
                self._emit(
                    "scan_planner_ready",
                    subgoal_index=self._mission.current_index,
                    traj_id=self._mission.ready_traj_id,
                )
            elif now - self._mission.goal_dispatched_at > self._plan_ready_timeout:
                self._mission.replan_fail_count += 1
                self._mission.stuck_redispatch_count += 1
                self.get_logger().warn(
                    f"ScanPlannerRun timeout ({self._plan_ready_timeout}s), "
                    f"fail={self._mission.replan_fail_count}/{self._replan_fail_threshold}, "
                    f"scale_retry={self._mission.stuck_redispatch_count}"
                )
                self._emit(
                    "replan_timeout",
                    subgoal_index=self._mission.current_index,
                    fail_count=self._mission.replan_fail_count,
                )
                if should_pause_replan(
                    replan_fail_count=self._mission.replan_fail_count,
                    pause_after=self._plan_fail_pause_after,
                ):
                    if self._maybe_enter_plan_fail_recovery("replan_timeout"):
                        return
                if self._mission.replan_fail_count >= self._replan_fail_threshold:
                    if self._maybe_enter_plan_fail_recovery("replan_fail_threshold"):
                        return
                    self._fail_mission(
                        "replan_fail_threshold",
                        subgoal_index=self._mission.current_index,
                    )
                    return
                self._dispatch_current_subgoal()
            return

        dist = self._dist_to_subgoal(self._mission.current_index)
        if dist is None:
            return

        if dist + 1e-3 < self._mission.best_dist_to_subgoal - self._progress_dist_delta:
            self._mission.best_dist_to_subgoal = dist
            self._mission.last_progress_at = now

        if (
            dist <= self._reach_m
            and now - self._mission.execute_entered_at >= self._min_execute_before_reach
        ):
            reached = True
        else:
            reached = False

        if reached:
            self.get_logger().info(
                f"SubgoalReached {self._mission.current_index + 1}/"
                f"{len(self._mission.waypoints)} offset={dist:.2f}m"
            )
            self._emit(
                "subgoal_reached",
                subgoal_index=self._mission.current_index,
                offset_m=round(dist, 3),
            )
            if self._semantic_task_kind == "search":
                self._on_search_frontier_reached(dist)
                return
            self._mission.current_index += 1
            self._mission.last_progress_at = now
            self._mission.replan_fail_count = 0
            self._mission.stuck_redispatch_count = 0
            self._mission.recovery_step = 0
            self._mission.glass_recovery_count = 0
            self._mission.best_dist_to_subgoal = float("inf")
            self._apply_slow_mode(False)
            if self._mission.current_index >= len(self._mission.waypoints):
                if self._return_home_pending:
                    self._start_return_home(self._return_home_pending)
                    return
                self._mission.status = BtStatus.SUCCEEDED
                self._mission.phase = ExecPhase.IDLE
                self._apply_slow_mode(False)
                if self._mission_mode == MissionMode.SEMANTIC_OBJNAV:
                    wp = self._mission.waypoints[0]
                    self._emit(
                        "object_found",
                        command_id=wp.command_id,
                        language_query=wp.language_query,
                        detail="semantic_nav_reached",
                        x=round(wp.x, 3),
                        y=round(wp.y, 3),
                    )
                    self._emit("mission_succeeded", detail="semantic_nav_reached")
                else:
                    self._emit("mission_succeeded", detail="all_subgoals")
                return
            self._dispatch_current_subgoal()
            return

        glass_stuck_s = min(self._stuck_timeout, self._glass_detour_timeout)
        stuck_limit = glass_stuck_s if self._is_glass_conservative() else self._stuck_timeout

        if now - self._mission.last_progress_at > stuck_limit:
            if (
                self._is_glass_conservative()
                and self._mission.recovery_step == 0
                and self._mission.glass_recovery_count < self._glass_max_recovery
            ):
                self._mission.recovery_step = 1
                self._mission.phase = ExecPhase.GLASS_HALT
                self._mission.recovery_until = now + self._glass_halt_wait
                self.get_logger().warn(
                    f"Glass suspected → HaltAndWait {self._glass_halt_wait}s "
                    f"(dist={dist:.2f}m)"
                )
                self._emit(
                    "glass_halt",
                    subgoal_index=self._mission.current_index,
                    distance_m=round(dist, 2),
                    glass_state=self._glass_state,
                )
                return
            if (
                self._is_glass_conservative()
                and self._mission.glass_recovery_count >= self._glass_max_recovery
            ):
                self._dispatch_backup_goal()
                return
            if self._mission.recovery_step == 0:
                self._mission.recovery_step = 1
                self._mission.phase = ExecPhase.RECOVERY_WAIT
                self._mission.recovery_until = now + self._recovery_wait_s
                self.get_logger().warn(
                    f"Subgoal stuck → Recovery Wait {self._recovery_wait_s}s "
                    f"(dist={dist:.2f}m)"
                )
                self._emit(
                    "recovery_wait",
                    subgoal_index=self._mission.current_index,
                    distance_m=round(dist, 2),
                )
                return
            if self._mission.stuck_redispatch_count < self._stuck_redispatch_max:
                if should_pause_replan(
                    replan_fail_count=self._mission.stuck_redispatch_count,
                    pause_after=self._plan_fail_pause_after,
                ):
                    if self._maybe_enter_plan_fail_recovery("scan_planner_stuck"):
                        return
                self._mission.stuck_redispatch_count += 1
                self._mission.recovery_step = min(self._mission.recovery_step + 1, 3)
                self.get_logger().warn(
                    f"Subgoal stuck, redispatch "
                    f"{self._mission.stuck_redispatch_count}/{self._stuck_redispatch_max} "
                    f"recovery_step={self._mission.recovery_step} (dist={dist:.2f}m)"
                )
                self._emit(
                    "subgoal_retry",
                    subgoal_index=self._mission.current_index,
                    retry=self._mission.stuck_redispatch_count,
                    recovery_step=self._mission.recovery_step,
                    distance_m=round(dist, 2),
                )
                self._mission.last_progress_at = now
                self._dispatch_current_subgoal()
                return
            if self._maybe_enter_plan_fail_recovery("scan_planner_stuck_exhausted"):
                return
            self._fail_mission(
                "scan_planner_stuck",
                subgoal_index=self._mission.current_index,
                distance_m=round(dist, 2),
                retries=self._mission.stuck_redispatch_count,
            )


def main() -> None:
    rclpy.init()
    node = DemoScanBtOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
