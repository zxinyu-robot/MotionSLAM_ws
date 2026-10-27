# -*- coding: utf-8 -*-
"""demo_scan_bt_orchestrator 的核心状态与纯工具（拆分出的第一部分）。

只包含：任务模式/状态枚举、同步改参客户端、子目标与任务状态数据结构、
waypoints yaml 加载。编排节点本体仍在 demo_scan_bt_orchestrator.py。
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import yaml
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from semantic_mission import SemanticSubgoal  # noqa: E402

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

