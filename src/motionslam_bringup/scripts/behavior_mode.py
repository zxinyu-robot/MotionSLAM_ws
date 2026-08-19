#!/usr/bin/env python3
"""边端物模型信封：按大疆上云 API 的 profile/properties/services/events 组织。

下行 method=behavior_mode.execute（:9879）
上行 method=behavior_mode.progress|behavior_mode.event（:9880）

data 七块：
  profile           所用机器人
  properties        所在场景 + 模式生命周期
  services          所用动作（技能，不是速度）
  policy            所参考的行为策略
  models            所用的模型
  events            所出现的异常行为日志（以上行为主）
  world_fragments   所生产的世界片段数据（以上行为主）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from task_context import TaskIdentityContext, validate_task_identity

SCHEMA = "motionslam.thing_envelope.v1"

METHOD_EXECUTE = "behavior_mode.execute"
METHOD_PROGRESS = "behavior_mode.progress"
METHOD_EVENT = "behavior_mode.event"
DOWNLINK_METHODS = frozenset({METHOD_EXECUTE})
UPLINK_METHODS = frozenset({METHOD_PROGRESS, METHOD_EVENT})
ALLOWED_METHODS = DOWNLINK_METHODS | UPLINK_METHODS

ALLOWED_PLATFORMS = frozenset({"unitree_go2_edu", "unitree_go2"})
ALLOWED_SKILLS = frozenset({"Navigate", "ReturnHome", "RecoverGlass", "ExploreFrontier"})
ALLOWED_MODE_IDS = frozenset(
    {
        "navigate_office",
        "navigate_intersection_cautious",
        "navigate_glass_aware",
        "return_home_on_link_lost",
        "explore_frontier",
    }
)
ALLOWED_SCENE_TYPES = frozenset(
    {
        "office_corridor",
        "intersection",
        "glass_facade",
        "elevator_lobby",
    }
)
ALLOWED_MODE_STATES = frozenset(
    {"INIT", "ARMED", "ACTIVE", "EXCEPTION", "TERMINATED"}
)
ALLOWED_RECOVERY_SKILLS = frozenset({"ReturnHome", "RecoverGlass", "none"})
ALLOWED_EXCEPTION_REASONS = frozenset(
    {"plan_fail", "stuck", "glass_trap", "link_lost", "robot_mismatch", "unresolved_goal"}
)
ALLOWED_FRAGMENT_LAYERS = frozenset(
    {
        "spatial_semantic_token",
        "spatial_voxel",
        "topology_subgraph",
        "perception_rgb",
        "pose_digest",
        "policy_archive",
        "exec_feedback",
    }
)
FORBIDDEN_CONTROL_KEYS = frozenset(
    {"cmd_vel", "/cmd_vel", "twist", "Twist", "initial_path", "/initial_path", "/motion/command"}
)
NAV_GOAL_TYPES = frozenset({"waypoint", "semantic_ref"})


@dataclass
class ThingEnvelopeValidation:
    ok: bool
    reason: str = ""
    envelope: Optional[dict[str, Any]] = None


def _norm(value: Any) -> str:
    return str(value).strip()


def _as_dict(value: Any, reject_code: str) -> tuple[Optional[dict[str, Any]], Optional[str]]:
    if not isinstance(value, dict):
        return None, reject_code
    return value, None


def _has_forbidden_control(obj: Any) -> Optional[str]:
    if isinstance(obj, dict):
        for key, val in obj.items():
            if str(key) in FORBIDDEN_CONTROL_KEYS:
                return f"REJECT_FORBIDDEN_CONTROL:{key}"
            nested = _has_forbidden_control(val)
            if nested:
                return nested
    elif isinstance(obj, list):
        for item in obj:
            nested = _has_forbidden_control(item)
            if nested:
                return nested
    return None


def flatten_identity(raw: dict[str, Any]) -> dict[str, Any]:
    """把大疆风格信封摊成现有 session/map/generation 身份字段。"""
    data = raw.get("data") if isinstance(raw.get("data"), dict) else {}
    props = data.get("properties") if isinstance(data.get("properties"), dict) else {}
    profile = data.get("profile") if isinstance(data.get("profile"), dict) else {}
    identity = dict(raw)
    identity["command_id"] = raw.get("tid") or raw.get("command_id")
    identity["session_id"] = raw.get("bid") or raw.get("session_id") or props.get("session_id")
    identity["map_id"] = props.get("map_id") or raw.get("map_id")
    identity["floor_id"] = props.get("floor_id") or raw.get("floor_id")
    identity["frame_id"] = props.get("frame_id") or raw.get("frame_id")
    identity["context_generation"] = props.get("context_generation", raw.get("context_generation"))
    identity["map_version"] = props.get("map_version", raw.get("map_version"))
    ts = raw.get("timestamp", raw.get("timestamp_ns", raw.get("created_ns")))
    if ts is not None:
        try:
            ts_i = int(ts)
        except (TypeError, ValueError):
            ts_i = None
        if ts_i is not None:
            identity["created_ns"] = ts_i if ts_i > 10**15 else ts_i * 1_000_000
    identity["robot_id"] = profile.get("sn") or profile.get("robot_id")
    identity["platform"] = profile.get("platform")
    return identity


def extract_nav_goal(envelope: dict[str, Any]) -> Optional[dict[str, Any]]:
    data = envelope.get("data") if isinstance(envelope.get("data"), dict) else {}
    services = data.get("services") if isinstance(data.get("services"), dict) else {}
    actions = services.get("actions") or []
    if not isinstance(actions, list):
        return None
    for action in actions:
        if not isinstance(action, dict):
            continue
        if _norm(action.get("skill")) != "Navigate":
            continue
        goal = action.get("goal")
        if isinstance(goal, dict):
            return goal
    return None


def extract_skills(envelope: dict[str, Any]) -> list[str]:
    data = envelope.get("data") if isinstance(envelope.get("data"), dict) else {}
    services = data.get("services") if isinstance(data.get("services"), dict) else {}
    actions = services.get("actions") or []
    out: list[str] = []
    if not isinstance(actions, list):
        return out
    for action in actions:
        if isinstance(action, dict) and action.get("skill"):
            out.append(_norm(action.get("skill")))
    return out


def _validate_profile(profile: dict[str, Any], ctx: TaskIdentityContext) -> Optional[str]:
    platform = _norm(profile.get("platform", ""))
    if not platform:
        return "REJECT_MISSING_ROBOT"
    if platform not in ALLOWED_PLATFORMS:
        return f"REJECT_ROBOT_PLATFORM:{platform}"
    sn = _norm(profile.get("sn") or profile.get("robot_id") or "")
    if not sn:
        return "REJECT_MISSING_ROBOT_ID"
    if ctx.local_platform and platform != _norm(ctx.local_platform):
        return "REJECT_ROBOT_MISMATCH"
    if ctx.local_robot_id and sn != _norm(ctx.local_robot_id):
        return "REJECT_ROBOT_MISMATCH"
    tags = profile.get("capability_tags")
    if tags is not None:
        if not isinstance(tags, list) or not tags:
            return "REJECT_CAPABILITY_TAGS"
        for tag in tags:
            if _norm(tag) not in ALLOWED_SKILLS:
                return f"REJECT_SKILL_UNSUPPORTED:{tag}"
    return None


def _validate_properties(props: dict[str, Any], *, require_scene: bool) -> Optional[str]:
    if require_scene:
        scene_type = _norm(props.get("scene_type", ""))
        if not scene_type:
            return "REJECT_MISSING_SCENE"
        if scene_type not in ALLOWED_SCENE_TYPES:
            return f"REJECT_SCENE_TYPE:{scene_type}"
        tags = props.get("semantic_tags")
        if tags is None or not isinstance(tags, list) or not tags:
            return "REJECT_MISSING_SCENE_TAGS"
    mode_state = props.get("mode_state")
    if mode_state is not None and _norm(mode_state).upper() not in ALLOWED_MODE_STATES:
        return f"REJECT_MODE_STATE:{mode_state}"
    return None


def _validate_goal(goal: Any, index: int) -> Optional[str]:
    if not isinstance(goal, dict):
        return f"REJECT_ACTION_{index}_GOAL_TYPE"
    gtype = _norm(goal.get("type", "waypoint")).lower()
    if gtype not in NAV_GOAL_TYPES:
        return f"REJECT_ACTION_{index}_GOAL_KIND:{gtype}"
    if gtype == "semantic_ref":
        if not _norm(goal.get("ref", "")):
            return f"REJECT_ACTION_{index}_MISSING_REF"
        has_xyz = all(k in goal for k in ("x", "y", "z"))
        if not has_xyz:
            return "REJECT_UNRESOLVED_GOAL"
        return None
    for key in ("x", "y", "z"):
        if key not in goal:
            return f"REJECT_ACTION_{index}_MISSING_{key.upper()}"
        try:
            float(goal[key])
        except (TypeError, ValueError):
            return f"REJECT_ACTION_{index}_BAD_{key.upper()}"
    return None


def _validate_services(services: dict[str, Any], profile: dict[str, Any]) -> Optional[str]:
    actions = services.get("actions")
    if not isinstance(actions, list) or not actions:
        return "REJECT_MISSING_ACTIONS"
    allowed = {_norm(t) for t in (profile.get("capability_tags") or ALLOWED_SKILLS)}
    for i, action in enumerate(actions):
        if not isinstance(action, dict):
            return f"REJECT_ACTION_{i}_TYPE"
        skill = _norm(action.get("skill", ""))
        if not skill:
            return f"REJECT_ACTION_{i}_MISSING_SKILL"
        if skill not in ALLOWED_SKILLS:
            return f"REJECT_SKILL_UNSUPPORTED:{skill}"
        if allowed and skill not in allowed:
            return f"REJECT_SKILL_UNSUPPORTED:{skill}"
        if skill == "Navigate":
            err = _validate_goal(action.get("goal"), i)
            if err:
                return err
    return None


def _validate_policy(policy: dict[str, Any]) -> Optional[str]:
    mode_id = _norm(policy.get("mode_id", ""))
    if not mode_id:
        return "REJECT_MISSING_BEHAVIOR_MODE"
    if mode_id not in ALLOWED_MODE_IDS:
        return f"REJECT_MODE_UNKNOWN:{mode_id}"
    version = policy.get("mode_version")
    if version is not None and not _norm(version):
        return "REJECT_MODE_VERSION"
    life = policy.get("lifecycle_policy")
    if life is not None:
        if not isinstance(life, dict):
            return "REJECT_LIFECYCLE_POLICY_TYPE"
        for key in ("on_link_lost", "on_glass_trap", "on_plan_fail"):
            if key not in life:
                continue
            skill = _norm(life.get(key))
            if skill not in ALLOWED_RECOVERY_SKILLS:
                return f"REJECT_LIFECYCLE_POLICY:{key}:{skill}"
    return None


def _validate_models(models: dict[str, Any]) -> Optional[str]:
    if not models:
        return "REJECT_MISSING_MODELS"
    for key in ("world_model", "planner", "controller"):
        block = models.get(key)
        if not isinstance(block, dict) or not _norm(block.get("name", "")):
            return f"REJECT_MISSING_MODEL:{key}"
    return None


def _validate_events(events: Any, *, required: bool) -> Optional[str]:
    if events is None:
        return "REJECT_MISSING_EVENTS" if required else None
    if not isinstance(events, list):
        return "REJECT_EVENTS_TYPE"
    if required and not events:
        return "REJECT_EMPTY_EVENTS"
    for i, ev in enumerate(events):
        if not isinstance(ev, dict):
            return f"REJECT_EVENT_{i}_TYPE"
        name = _norm(ev.get("event") or ev.get("method") or "")
        if not name:
            return f"REJECT_EVENT_{i}_MISSING_NAME"
        reason = ev.get("reason")
        if reason is not None and _norm(reason) not in ALLOWED_EXCEPTION_REASONS:
            return f"REJECT_EVENT_{i}_REASON:{reason}"
    return None


def _validate_world_fragments(fragments: Any, *, required: bool) -> Optional[str]:
    if fragments is None:
        return "REJECT_MISSING_WORLD_FRAGMENTS" if required else None
    if not isinstance(fragments, list):
        return "REJECT_WORLD_FRAGMENTS_TYPE"
    if required and not fragments:
        return "REJECT_EMPTY_WORLD_FRAGMENTS"
    for i, frag in enumerate(fragments):
        if not isinstance(frag, dict):
            return f"REJECT_FRAGMENT_{i}_TYPE"
        layer = _norm(frag.get("layer_id", ""))
        if layer not in ALLOWED_FRAGMENT_LAYERS:
            return f"REJECT_FRAGMENT_{i}_LAYER:{layer}"
        if not (frag.get("ref") or frag.get("digest") or frag.get("uri")):
            return f"REJECT_FRAGMENT_{i}_MISSING_REF"
        if "payload" in frag or "points" in frag or "cloud" in frag:
            return f"REJECT_FRAGMENT_{i}_INLINE_PAYLOAD"
    return None


def validate_thing_envelope(
    raw: dict[str, Any],
    ctx: TaskIdentityContext,
    *,
    require_behavior_mode: bool = True,
) -> ThingEnvelopeValidation:
    schema = _norm(raw.get("schema") or raw.get("schema_version") or "")
    if schema and schema != SCHEMA:
        return ThingEnvelopeValidation(False, f"REJECT_SCHEMA:{schema}")
    if not schema:
        return ThingEnvelopeValidation(False, "REJECT_SCHEMA:")

    forbidden = _has_forbidden_control(raw)
    if forbidden:
        return ThingEnvelopeValidation(False, forbidden)

    method = _norm(raw.get("method", METHOD_EXECUTE))
    if method not in ALLOWED_METHODS:
        return ThingEnvelopeValidation(False, f"REJECT_METHOD:{method}")

    if not (raw.get("tid") or raw.get("command_id")):
        return ThingEnvelopeValidation(False, "REJECT_MISSING_COMMAND_ID")
    if not (raw.get("bid") or raw.get("session_id")):
        return ThingEnvelopeValidation(False, "REJECT_MISSING_BID")

    data, err = _as_dict(raw.get("data"), "REJECT_MISSING_DATA")
    if err or data is None:
        return ThingEnvelopeValidation(False, err or "REJECT_MISSING_DATA")

    identity = flatten_identity(raw)
    identity_err = validate_task_identity(
        identity,
        ctx,
        require_lifecycle=method in DOWNLINK_METHODS,
        check_context_generation=True,
    )
    if identity_err:
        return ThingEnvelopeValidation(False, identity_err)

    profile, err = _as_dict(data.get("profile"), "REJECT_MISSING_ROBOT")
    if err or profile is None:
        return ThingEnvelopeValidation(False, err or "REJECT_MISSING_ROBOT")
    err = _validate_profile(profile, ctx)
    if err:
        return ThingEnvelopeValidation(False, err)

    props, err = _as_dict(data.get("properties"), "REJECT_MISSING_SCENE")
    if err or props is None:
        return ThingEnvelopeValidation(False, err or "REJECT_MISSING_SCENE")
    err = _validate_properties(props, require_scene=True)
    if err:
        return ThingEnvelopeValidation(False, err)

    policy, err = _as_dict(data.get("policy"), "REJECT_MISSING_BEHAVIOR_MODE")
    if err or policy is None:
        return ThingEnvelopeValidation(False, err or "REJECT_MISSING_BEHAVIOR_MODE")
    if require_behavior_mode:
        err = _validate_policy(policy)
        if err:
            return ThingEnvelopeValidation(False, err)

    models, err = _as_dict(data.get("models"), "REJECT_MISSING_MODELS")
    if err or models is None:
        return ThingEnvelopeValidation(False, err or "REJECT_MISSING_MODELS")
    err = _validate_models(models)
    if err:
        return ThingEnvelopeValidation(False, err)

    if method in DOWNLINK_METHODS:
        services, err = _as_dict(data.get("services"), "REJECT_MISSING_ACTIONS")
        if err or services is None:
            return ThingEnvelopeValidation(False, err or "REJECT_MISSING_ACTIONS")
        err = _validate_services(services, profile)
        if err:
            return ThingEnvelopeValidation(False, err)
        if data.get("events"):
            return ThingEnvelopeValidation(False, "REJECT_EVENTS_ON_SERVICE")
        if "world_fragments" in data:
            err = _validate_world_fragments(data.get("world_fragments"), required=False)
            if err:
                return ThingEnvelopeValidation(False, err)
    else:
        mode_state = _norm(props.get("mode_state", "")).upper()
        if not mode_state:
            return ThingEnvelopeValidation(False, "REJECT_MISSING_MODE_STATE")
        err = _validate_events(data.get("events"), required=mode_state == "EXCEPTION")
        if err:
            return ThingEnvelopeValidation(False, err)
        if "world_fragments" in data:
            err = _validate_world_fragments(data.get("world_fragments"), required=False)
            if err:
                return ThingEnvelopeValidation(False, err)

    return ThingEnvelopeValidation(True, "ACCEPT", raw)


def build_progress_envelope(
    *,
    tid: str,
    bid: str,
    timestamp_ns: int,
    profile: dict[str, Any],
    properties: dict[str, Any],
    policy: dict[str, Any],
    models: dict[str, Any],
    events: Optional[list[dict[str, Any]]] = None,
    world_fragments: Optional[list[dict[str, Any]]] = None,
    method: str = METHOD_PROGRESS,
) -> dict[str, Any]:
    data: dict[str, Any] = {
        "profile": profile,
        "properties": properties,
        "policy": policy,
        "models": models,
        "events": events or [],
    }
    if world_fragments:
        data["world_fragments"] = world_fragments
    return {
        "schema": SCHEMA,
        "tid": tid,
        "bid": bid,
        "timestamp": timestamp_ns,
        "method": method,
        "need_reply": 1 if (events and properties.get("mode_state") == "EXCEPTION") else 0,
        "data": data,
    }
