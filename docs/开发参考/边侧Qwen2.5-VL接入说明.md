# 边侧 Qwen2.5-VL-7B 接入说明

> 给边侧（世界空间）接入 **Qwen2.5-VL-7B** 用：**端侧上行能生产什么**、**JSON 谁构建**、**TCP 必须下发什么**。  
> 契约实现：[behavior_mode.py](../../src/motionslam_bringup/scripts/behavior_mode.py) · 端口：[edge_uplink.yaml](../../src/motionslam_bringup/config/edge_uplink.yaml)  
> 总规范：[边端协同接口规范.md](../项目规范/边端协同接口规范.md)

Qwen2.5-VL-7B 是视觉语言模型：吃 **图像 + 文本**，产出场景/行为决策。它 **不得** 下发速度或折线。边侧服务负责：解码端侧物料 → 组 VL 输入 → 把模型输出 **装配成** `motionslam.thing_envelope.v1` → TCP 打到狗端 `:9879`（一行一个 JSON，`\n` 结束）。

日常 `nav start` **不起** 边端。联调必须：

```bash
WITH_EDGE_UPLINK=true WITH_SEMANTIC_OBJNAV=true ./scripts/nav/start_demo_scan_nav.sh
```

边侧先起监听（默认 `EDGE_HOST=${EDGE_HOST}`），再开栈。参考：`python3 scripts/offline/mock_edge_uplink_receiver.py`。

---

## 1. 端侧上行数据（给 VL 的物料）

狗端 Orin **主动连边侧 IP**（默认 `${EDGE_HOST}`，launch `edge_host`；**不是边侧连狗**）。坐标系 `world`（Super-LIO ENU）。身份默认：`session_id=demo_live`，`map_id=demo_live_map`，`floor_id=floor_01`，`sn=go2_001`。

**TCP 上行总表**（`WITH_EDGE_UPLINK=true` 时；9876 另需 `with_semantic_bev`）：

| 端口 | 节点 | 方向 | 形态 | schema / 魔数 | Qwen 用途 |
|------|------|------|------|---------------|-----------|
| **9878** | `rgb_keyframe_uplink` | GO2→Edge | 二进制 `MSRGB` | `motionslam.rgb_forward.v1` | **主图像输入** |
| **9877** | `subgraph_publisher` | GO2→Edge | JSON Lines | `motionslam.sparse_subgraph.v1` | 位姿 + frontier xyz |
| **9880** | `execution_feedback` | GO2→Edge | JSON Lines | `execution_feedback.v1` + 心跳 | 任务/异常闭环 |
| **9876** | `semantic_token_uplink` | GO2→Edge | 二进制 `MSTOK` | `spatial_semantic_token.v1` | 预留（非 Demo1 必须） |
| **9879** | `directive_receiver` | Edge→GO2 | JSON Lines | `thing_envelope.v1` | **下行，非上行** |

点云、`/global_path`、`/initial_path`、`/cmd_vel` **不上行、也禁止下行**。

### 1.1 RGB（9878）— Qwen 的图像

帧格式：`magic(5)=MSRGB` + `ver u8` + `json_len u32be` + `payload_len u32be` + meta JSON + payload。解包：`rgb_forward_frame.unpack_rgb_forward_frame`。

meta 要点：

```json
{
  "schema": "motionslam.rgb_forward.v1",
  "layer_id": "perception_rgb",
  "session_id": "demo_live",
  "floor_id": "floor_01",
  "map_id": "demo_live_map",
  "stream_field": "video360p",
  "seq": 12,
  "timestamp_ns": 1755590000000000000,
  "subgraph_version": 3,
  "context_generation": 0,
  "payload_len": 4096,
  "pose": {"x": 1.2, "y": -0.3, "z": 0.35, "yaw_deg": 12.5}
}
```

`payload` 是机身 `Go2FrontVideoData.video360p` **原样**（常见为 JPEG 或 H264 片段，边侧自行解码成 RGB 再喂 Qwen）。解码失败就丢帧，不要用坏图硬推 Navigate。

### 1.2 子图（9877）— 文本空间上下文

一行一个 JSON。VL 用 `robot_pose` 和 `frontiers[].{x,y,z}` 填目标点（端侧 **不解** `semantic_ref`，下行 goal 必须带 xyz）。

```json
{
  "schema": "motionslam.sparse_subgraph.v1",
  "layer_id": "topology_subgraph",
  "session_id": "demo_live",
  "map_id": "demo_live_map",
  "floor_id": "floor_01",
  "version": 3,
  "context_generation": 0,
  "trigger_reason": "periodic",
  "pgo_state": "IDLE",
  "robot_pose": {"x": 1.2, "y": -0.3, "z": 0.35, "yaw_deg": 12.5},
  "nodes": [{"node_id": "kf_1", "x": 0.0, "y": 0.0, "z": 0.35, "rgb_keyframe_ref": "rgb_1"}],
  "edges": [],
  "frontiers": [
    {"frontier_id": "f_0", "x": 3.1, "y": 0.2, "z": 0.35, "geom_score": 0.8}
  ]
}
```

无关键帧时 `nodes` 可能只有 `"robot"`。`pgo_state=OPTIMIZING` 时端侧会拒新任务。

### 1.3 执行反馈（9880）— 再规划用

节点：`execution_feedback_node` · 构建：`build_execution_feedback_v1`（[execution_feedback.py](../../src/motionslam_bringup/scripts/execution_feedback.py)）· 订阅 `/demo/mission/event`。

**形态**：JSON Lines，一行一条。除任务反馈外，同端口还会周期性发 **链路心跳**（见 §1.5）。

**载荷字段（`motionslam.execution_feedback.v1`）**：

| 字段 | 说明 |
|------|------|
| `schema` | 固定 `motionslam.execution_feedback.v1` |
| `layer_id` | `exec_feedback` |
| `session_id` / `map_id` / `floor_id` / `tile_id` | 与上行身份一致 |
| `context_generation` | 与 LIO 代数对齐 |
| `command_id` | 对应下行 `tid`（若有） |
| `event` | 见下表 |
| `reason` | 拒单时为 `REJECT_*`；异常时为事件名 |
| `safety_state` | `OK` / `DEGRADED` / `HOLD` |
| `robot_pose` | `{x,y,z,yaw_deg}` |
| `mode_id` / `mode_state` / `scene_type` | 物模型信封接受后可选带上 |
| `events` / `world_fragments` | 规范预留；**当前上行仍以扁平 `event` 为主**，完整七块物模型上行见 `build_progress_envelope`（尚未接入本节点） |

**常见 `event`（来自 BT / command_executor）**：

| event | 含义 |
|-------|------|
| `task_accepted` | 物模型/ActionGroup 校验通过 |
| `task_rejected` | 校验失败，看 `reason` |
| `search_navigate` | Navigate 已武装，将发 semantic goal |
| `search_explore` | ExploreFrontier 武装 |
| `subgoal_dispatched` | BT 已发 `/goal_pose` |
| `subgoal_reached` | 子目标到达 |
| `plan_fail_hold` | 规划失败挂起 |
| `glass_trap` | 玻璃陷阱 |
| `return_home_started` / `return_home_done` | 返航 |
| `lifecycle_active` / `lifecycle_inactive` | 栈 lifecycle |

代码 **没有** `pct_nav_started`。Qwen 闭环应监听 `task_rejected`、`glass_trap`、`plan_fail_hold` 等，再发新 `execute`（新 `tid`）。

### 1.4 Token（9876）— 可选

节点：`semantic_token_uplink_node` · 构建：`build_semantic_token_meta` + 二进制 `MSTOK` 帧（[semantic_bev_frame.py](../../src/motionslam_bringup/scripts/semantic_bev_frame.py)）。

**条件**：`with_semantic_bev:=true` 且 BEV 真网络在线；无 TensorRT engine 时 **不要** 当 Qwen 输入依赖。Demo 1 主链用 **9878 RGB + 9877 子图** 即可。

### 1.5 9880 心跳

同 `execution_feedback_node`，周期性 enqueue：

```json
{"schema": "motionslam.uplink_heartbeat.v1", "event": "heartbeat"}
```

端内发布 `/demo/mission/uplink_ok`（链路是否通）；BT 可据此断网返航。边侧 mock 可忽略，但应能解析 JSON Lines 混流。

### 1.6 仅端内 ROS、不发边侧 TCP

| 话题 | 内容 |
|------|------|
| `/edge/data_layer/catalog` | 各数据层 READY/STALE 快照（`data_layer_registry_node`） |
| `/edge/uplink/rgb_event`、`/edge/uplink/subgraph_event` | 本机 uplink 调试 |
| `/edge/data_layer/layer_event` | 层 publish 事件 |
| `/demo/mission/uplink_ok` | 9880 心跳是否成功 |

边侧 **listen 9876–9878、9880** 即可收 TCP 上行；catalog 需另订阅 ROS 或暂不使用。

### 1.7 对 Qwen 的上行用法（小结）

| 用途 | 端口 |
|------|------|
| **图像** | **9878**（解码 `video360p` → VL 输入） |
| **空间文本**（位姿、frontier xyz） | **9877** |
| **执行结果 / 拒单 / 异常** | **9880** |
| BEV Token | 9876（预留，Demo 1 非必须） |

### 1.8 端侧本机身份（下行必须对齐）

| 字段 | 默认 | 不一致后果 |
|------|------|------------|
| `bid` / session | `demo_live` | `REJECT_SESSION` |
| `map_id` | `demo_live_map` | `REJECT_MAP` |
| `floor_id` | `floor_01` | `REJECT_FLOOR` |
| `profile.sn` | `go2_001` | `REJECT_ROBOT_MISMATCH` |
| `profile.platform` | `unitree_go2_edu` | 同上 |
| `frame_id` | `world` | `REJECT_FRAME` |
| `context_generation` | 抄上行；未知则 **省略** | 大于端侧当前值 → `REJECT_CONTEXT` |
| `timestamp` | **当前时间** ns 或 ms | `0` 会 `REJECT_EXPIRED`（默认 TTL 10s） |

建议信封带 `"ttl_ms": 30000`。

---

## 2. JSON 由谁构建（上行 / 下行）

**端侧不会用模型生成下行 JSON。** 只有校验与转发；上行物料由各节点 **纯函数组包**。

| 方向 | 谁构建 | 实现 | 现状 |
|------|--------|------|------|
| **下行 :9879** | **边侧**（模板 + 装配器） | 手写 JSON 或读 [thing_envelope_qwen_navigate.json](../../scripts/offline/thing_envelope_qwen_navigate.json) + [send_thing_envelope.py](../../scripts/offline/send_thing_envelope.py) | 端侧无 `build_execute_envelope()`；`validate_thing_envelope` 只校验 |
| **上行 :9877** | 端侧 `subgraph_publisher` | `build_sparse_subgraph_v1` | `WITH_EDGE_UPLINK=true` 时约 1 Hz |
| **上行 :9878** | 端侧 `rgb_keyframe_uplink` | `build_rgb_forward_meta` + `pack_rgb_forward_frame` | 随前向视频帧 |
| **上行 :9880** | 端侧 `execution_feedback` | `build_execution_feedback_v1` | 随 mission 事件 + 心跳 |
| **上行 :9876** | 端侧 `semantic_token_uplink` | `build_semantic_token_meta` | 需 `with_semantic_bev` |
| **板载导航** | yaml 航点 | `acceptance_short_waypoints.yaml` 等 | **不经 JSON** |

**数据流**：

```text
端侧生产（9878 图 + 9877 子图 + 9880 反馈）
        → 边侧 Qwen2.5-VL-7B + 装配器
        → 下行 thing_envelope.v1（9879）
        → directive_receiver 校验
        → command_executor 抽 Navigate xyz
        → BT /demo/mission/semantic_goal → PCT/SCAN
        → 9880 再上行（闭环）
```

### 2.1 下行信封结构（边侧组装目标）

大疆式：**外层 + `data` 七块**。发送：**一行 JSON + `\n`**。

```text
{
  schema: "motionslam.thing_envelope.v1"
  tid, bid, timestamp, ttl_ms
  method: "behavior_mode.execute"
  data:
    profile       ← 边侧写死 sn/platform（勿让模型编）
    properties    ← scene_type / semantic_tags / map_id / mode_state
    services      ← actions[]: skill + goal{x,y,z}（xyz 必填）
    policy        ← mode_id + lifecycle_policy
    models        ← world_model=Qwen2.5-VL-7B, planner, controller
    （禁止 data.events）
}
```

`send_thing_envelope.py` 运行时只改：`tid`（新 uuid）、`timestamp`（`time.time_ns()`）、可选 goal 的 `x/y/z`。模板里 `timestamp: 0` **不能直接发**（会 `REJECT_EXPIRED`）。

端侧收到后：`directive_receiver` → 校验 → 原样发 `/semantic/action_group` → `command_executor` → `/demo/mission/semantic_goal`。

### 2.2 上行 JSON / 帧结构（端侧已实现的 builder）

| 端口 | builder | schema |
|------|---------|--------|
| 9877 | `build_sparse_subgraph_v1` | `motionslam.sparse_subgraph.v1` |
| 9878 meta | `build_rgb_forward_meta` | `motionslam.rgb_forward.v1`（+ MSRGB 二进制 payload） |
| 9880 | `build_execution_feedback_v1` | `motionslam.execution_feedback.v1` |
| 9876 meta | `build_semantic_token_meta` | `motionslam.spatial_semantic_token.v1`（+ MSTOK payload） |

规范中的完整物模型 **上行** `build_progress_envelope`（[behavior_mode.py](../../src/motionslam_bringup/scripts/behavior_mode.py)）已定义，**尚未**接到 `execution_feedback_node`；当前 9880 仍是扁平 `execution_feedback.v1`。

### 2.3 旧下行（兼容，边侧接 Qwen 不要用）

`require_behavior_mode` 默认 **false** 时仍接受 `motionslam.action_group.v1`：

```text
schema, command_id, session_id, task_type, stages[]{ NAVIGATE + waypoints{x,y,z} }
```

边侧接 Qwen **应只发** `thing_envelope.v1`。mock 旧路径：`scripts/offline/mock_semantic_directive.py --kind action_group`。

---

## 3. 建议的 VL 流水（不要让 7B 直接生成整封物模型）

7B 很难稳定吐出七块信封。边侧拆两步：

```text
RGB 解码图 + 文本上下文
        → Qwen2.5-VL-7B
        → 紧凑决策 JSON（只含场景/模式/目标意图）
        → 边侧装配器（填 profile / models / xyz / tid）
        → TCP :9879  thing_envelope.v1
```

### 3.1 喂给 Qwen 的文本（示例）

把最新 RGB 当图；文本用中英均可，但 **枚举必须用仓库英文 id**：

```text
你是 Go2 边侧决策器。只根据图像与空间上下文选择行为模式，不要输出速度或路径点序列。

机器人: unitree_go2_edu sn=go2_001
pose_world: x=… y=… z=… yaw_deg=…
frontiers: [{id,x,y,z,score}, …]
user_query: （可选，如「前面路口右转」）

只输出一个 JSON 对象，字段：
- scene_type: office_corridor | intersection | glass_facade
- semantic_tags: 字符串数组，至少 1 个
- mode_id: navigate_office | navigate_intersection_cautious | navigate_glass_aware | return_home_on_link_lost | explore_frontier
- skill: Navigate | ReturnHome | RecoverGlass | ExploreFrontier
- goal_frontier_id: 选 frontiers 里的 id，或 null
- adaptation_rationale: 一句中文
禁止输出 cmd_vel、twist、initial_path。
```

`elevator_lobby` 可被校验接受，但 Demo 1 没有电梯技能，不要选。

### 3.2 装配器必须补上的块

| 块 | 谁填 |
|----|------|
| `profile` | 边侧写死本机，不要让模型编 sn |
| `properties.map_id/floor_id/frame_id/context_generation` | 抄上行 |
| `services.actions[].goal.x/y/z` | 用选中的 frontier，或 pose 前方 1～2 m（z 用当前 `pose.z`） |
| `policy.lifecycle_policy` | 固定 `on_link_lost=ReturnHome`，`on_glass_trap=RecoverGlass` |
| `models` | 见下一节，`world_model.name` 必须是实际模型名 |

`skill=Navigate` 且只有 `semantic_ref`、没有 xyz → `REJECT_UNRESOLVED_GOAL`。

---

## 4. 边侧必须下发的 JSON（现网会校验）

- 狗端监听 `0.0.0.0:9879`
- 一条指令 = 一个 JSON 对象 + `\n`
- `method` 只能是 `behavior_mode.execute`
- **不要**带 `data.events`（否则 `REJECT_EVENTS_ON_SERVICE`）

可发送样例（把 `timestamp` 换成 `time.time_ns()`，xyz 换成子图/推算值）：

```json
{
  "schema": "motionslam.thing_envelope.v1",
  "tid": "qwen-20260819-0001",
  "bid": "demo_live",
  "timestamp": 1755590400000000000,
  "ttl_ms": 30000,
  "method": "behavior_mode.execute",
  "data": {
    "profile": {
      "sn": "go2_001",
      "platform": "unitree_go2_edu",
      "domain": "legged",
      "type": "go2",
      "thing_version": "1.0.0",
      "capability_tags": ["Navigate", "ReturnHome", "RecoverGlass", "ExploreFrontier"]
    },
    "properties": {
      "scene_type": "office_corridor",
      "semantic_tags": ["forward_along_corridor"],
      "language_query": "",
      "map_id": "demo_live_map",
      "floor_id": "floor_01",
      "frame_id": "world",
      "mode_state": "INIT"
    },
    "services": {
      "actions": [
        {
          "skill": "Navigate",
          "goal": {
            "type": "waypoint",
            "x": 2.0,
            "y": 0.0,
            "z": 0.35,
            "yaw_rad": 0.0
          }
        }
      ]
    },
    "policy": {
      "mode_id": "navigate_office",
      "mode_version": "1",
      "policy_ref": "qwen2.5-vl-7b",
      "lifecycle_policy": {
        "on_link_lost": "ReturnHome",
        "on_glass_trap": "RecoverGlass"
      },
      "adaptation_rationale": "走廊通视，选常规室内导航模式"
    },
    "models": {
      "world_model": {"name": "Qwen2.5-VL-7B", "version": "7b"},
      "planner": {"name": "pct_resident", "version": "cpp"},
      "controller": {"name": "scan_navi_mode_3"}
    }
  }
}
```

路口样例：把 `scene_type` 改为 `intersection`，`semantic_tags` 改为 `["turn_right_at_next_junction"]`，`mode_id` 改为 `navigate_intersection_cautious`，`language_query` 可填用户原话；**xyz 仍要边侧算好**。

玻璃样例：`scene_type=glass_facade`，`mode_id=navigate_glass_aware`。

### 4.1 允许枚举（写错即拒）

| 字段 | 允许值 |
|------|--------|
| `platform` | `unitree_go2_edu`、`unitree_go2` |
| `scene_type` | `office_corridor`、`intersection`、`glass_facade`（`elevator_lobby` 预留） |
| `mode_id` | `navigate_office`、`navigate_intersection_cautious`、`navigate_glass_aware`、`return_home_on_link_lost`、`explore_frontier` |
| `skill` | `Navigate`、`ReturnHome`、`RecoverGlass`、`ExploreFrontier` |
| `goal.type` | `waypoint`；`semantic_ref` 必须同时有 `ref` 和 `x,y,z` |

`models` 三个 `name` 均必填：`world_model`、`planner`、`controller`。planner/controller 填端侧真实栈，不要填 Nav2。

### 4.2 发送

```python
import json, socket, time
obj["timestamp"] = time.time_ns()
s = socket.create_connection(("${GO2_IP}", 9879), timeout=3)  # 狗端 Orin IP
s.sendall((json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8"))
```

仓库样例文件：[scripts/offline/thing_envelope_qwen_navigate.json](../../scripts/offline/thing_envelope_qwen_navigate.json)  
发送脚本：`python3 scripts/offline/send_thing_envelope.py --host <GO2_IP> --file scripts/offline/thing_envelope_qwen_navigate.json`

成功：`:9880` 出现 `task_accepted`，随后 `search_navigate`；狗端 BT 发 `/demo/mission/semantic_goal`。  
失败：`task_rejected` + `REJECT_*`。常见：过期、错 sn、缺 scene/mode/models、带了 `twist`、`context_generation` 超前、Navigate 无 xyz。

栈须 `lifecycle=active`，否则 `REJECT_LIFECYCLE`。

---

## 5. 模型职责边界

| 由 Qwen2.5-VL-7B 做 | 不要做 |
|---------------------|--------|
| 从图像判断 `scene_type` / `semantic_tags` / `mode_id` | 输出 `/cmd_vel`、折线、Nav2 控制器名 |
| 读 `user_query`（语音 ASR 文本）译成 tags | 在端侧 PCT 里做语义 |
| 在 frontier 列表里选一个几何目标 id | 自己发明与 `world` 对不齐的坐标（装配器应用子图坐标） |
| 写一句 `adaptation_rationale` | 预填 `events` 异常日志 |

闭环（RT-2）：下一帧 Qwen 结合 `:9880` 的 `glass_trap` / `plan_fail_hold` 再出一封新 `execute`（新 `tid`）。本轮端侧不会根据异常自动改 `mode_id`。
