# 云端闭环：vkernel 虚拟设备内核

目标架构在云上跑通：把"机器人 Linux"虚拟化——虚拟雷达/里程计/执行器 +
Thin Harness 任务意图入口，上层链路（PCT→BT→SCAN→forwarder）零改动。

## 组成

| 件 | 文件 | 作用 |
|---|---|---|
| vkernel_sim | `src/motionslam_bringup/scripts/vkernel_sim.py` | 虚拟设备内核：合成 `/lio/cloud_world`（房间点云）+ `/lio/robo/odom`（按 `/cmd_vel` 积分），可选接 `/api/sport/request` |
| thin_harness | `scripts/thin_harness_node.py` | Task Intent 下行 `:9879` / Execution Feedback 上行 `:9880`，桥到 BT 语义任务入口 |
| task_intent | `scripts/task_intent.py` | 意图校验纯逻辑（schema、白名单技能、禁控字段、world_ref 门禁），可 pytest |
| task_intent_client | `scripts/task_intent_client.py` | 云端客户端：发 intent→收 feedback→等到 `task_terminal` |
| cloud_mission_watch | `scripts/cloud_mission_watch.py` | 冒烟判据：等 `mission_succeeded` |
| 配置 | `config/vkernel_sim.yaml`、`config/demo_scan_planner_cloud.yaml` | 房间几何 / SCAN 动力学放宽 overlay |
| 镜像/脚本 | `docker/Dockerfile.cloud`、`scripts/dev/cloud_bootstrap.sh`、`scripts/dev/cloud_sim.sh`、`scripts/dev/source_cloud_env.sh` | x86 ROS Humble 镜像、依赖克隆+编译、一键起栈 |

## 跑法

```bash
docker build -f docker/Dockerfile.cloud -t motionslam:cloud .
docker run -d --name motionslam-cloud --network host --ipc host \
  -v "$PWD":/ws motionslam:cloud sleep infinity
docker exec motionslam-cloud bash /ws/scripts/dev/cloud_bootstrap.sh   # 一次性：clone+build

# 闭环 demo（vkernel + PCT→BT→SCAN→forwarder）
docker exec -d motionslam-cloud bash -lc "bash /ws/scripts/dev/cloud_sim.sh false > /ws/.sim.log 2>&1"
docker exec motionslam-cloud bash -lc \
  "source /ws/scripts/dev/source_cloud_env.sh && python3 /ws/src/motionslam_bringup/scripts/cloud_mission_watch.py"

# 目标架构任务意图闭环（thin harness）
docker exec -d motionslam-cloud bash -lc "bash /ws/scripts/dev/cloud_sim.sh true > /ws/.sim_thin.log 2>&1"
docker exec motionslam-cloud python3 /ws/src/motionslam_bringup/scripts/task_intent_client.py \
  --goal 2.0 0.6 0.35 --timeout 240
```

判据：`cloud_mission_watch` 退出码 0（`mission_succeeded`，15/15 子目标）；
`task_intent_client` 打印 `task_terminal state=succeeded`。

## 已知差异（相对真机）

- 无传感器/无 Super-LIO：`/lio/cloud_world`、`/lio/robo/odom` 由 vkernel 直接给出（RELIABLE QoS）。
- SCAN-Planner 用 `ros2-community` 分支头 + motionslam patch；
  THIRD_PARTY.md 记录的 pin `d62de08` 是 Jetson 本地提交，上游不存在；
  patch 有 1 处 `closed_loop_controller.cpp` hunk 与上游当前实现冲突，云端可跳过。
- `demo_scan_planner_cloud.yaml` 放宽 SCAN `max_acc/acc_tolerance`
  （真机姿态控制有冗余，纯积分器拐角动力学更苛刻）。
- launch 新增开关：`with_sensors/with_lio/with_mock_robot/with_thin_harness/extra_params_file`，真机默认全开、与现有行为一致。

## 跨楼层仿真（vkernel 高度场）

`vkernel_sim` 支持多楼层高度场：`walls` 每面墙带基准层高 `z0`（5 元组），
`floors=[x1,y1,x2,y2,z]` 平台、`ramps=[x1,y1,x2,y2,z1,z2,axis]` 坡道线性插值，
`odom.z = height_at(x,y)`。示例地图 `config/vkernel_multifloor.yaml`：
一楼房间 [0,4]²（东墙门洞 y∈[0.6,2.0]）→ 开敞坡道 x∈[4,7] 升 0→1.0 →
二楼平台 [7,11]×[0,4]（墙 z0=1.0）。航点：
`config/demo_scan_waypoints_multifloor.yaml`（一楼→爬坡→二楼绕圈→原路返回，11 个子目标）。

```bash
docker exec -d motionslam-cloud bash -lc \
  "bash /ws/scripts/dev/cloud_sim_multifloor.sh > /ws/.sim_mf.log 2>&1"
# 录像（订阅点云+里程计+体素占据，mission_succeeded 后自动渲染 mp4，
# 并存 npz 供离线反复渲染）
docker exec -d motionslam-cloud bash -lc \
  "source /ws/scripts/dev/source_cloud_env.sh && \
   python3 /ws/src/motionslam_bringup/scripts/cloud_video_render_3d.py \
     --out /ws/cross_floor_3d.mp4 --save-npz /ws/cross_floor.npz --timeout 480"
```

成果视频：`docs/media/cross_floor_demo.mp4`（双视口渲染版：
左 = 实时点云 `/lio/cloud_world` + 高度场地表点云，Foxglove 风格
高度着色；右 = 在线体素占据 `/grid_map/occupancy_inflate` 实录
（方块体素、按占据高度彩虹着色）+ 紫色轨迹带 + 白色航点球。
由 `scripts/cloud_video_render_3d.py --style dual` 订阅
`/lio/cloud_world` + `/lio/robo/odom` + `/grid_map/occupancy_inflate`
录制，PCT→BT→SCAN→forwarder 全链路真闭环；
`--save-npz/--load-npz` 支持录制一次后离线反复调渲染风格）。
另有 `scripts/cloud_video_render.py`（2D 俯视+侧视图）。

注意事项：
- PCT 是 0.2m 栅格 + 0.35m Chebyshev 膨胀：门洞净宽需 ≥1.2m，
  航点尽量放栅格中心（`k*0.2+0.1`），且 `max_goal_snap_m` 仅 0.15，
  目标落在膨胀格会被静默拒绝（`poses=0`）。
- SCAN 全局轨迹是 `PolynomialTraj::minSnapTraj`：**一串共线等距点会让拟合
  数值奇异**（系数爆到 1e11，`getPosition` 返回天文坐标 → 全判 out-of-map →
  局部目标塌回起点，`Close to goal` 死循环）。编排层已用
  `decimate_collinear_xy` 把直线段抽稀成首尾两点（走稳定的单段轨迹），
  并丢弃贴近 odom 的前导点（避免零长首段）。真机若复现同类现象同理处理。
- SCAN 障碍 z 过滤是相对地面 `ground_height=-0.45` 的窗口
  `obstacle_height_min/max`（真机 0.32/1.50 → 绝对 [-0.13, 1.05]）：
  二楼墙体（z≥1.3）默认对 SCAN 不可见。云端 overlay
  `demo_scan_planner_cloud.yaml` 已把 `obstacle_height_max` 放宽到 3.0，
  楼上体素才能进 `/grid_map/occupancy_inflate`；
  跨楼层真机更合理的做法是把高度过滤改成相对机器人/分层地图。

## 地图模式契约（map_mode: explore | prior）

任务 YAML `ros__parameters.map_mode`（或 adapter ROS 参数 `map_mode` 覆盖）声明本次
任务是否有先验图，fsr_mission_adapter 把它透传进每条 `/demo/mission/event` JSON。

- `explore`（默认）：无先验图。PCT 只吃滑动窗口内的已探索点云
  （/local_map/cloud 或 /lio/cloud_world），远距离目标靠
  frontier/语义候选逐段推进；编排模型不做长距离零样本路径生成。
- `prior`：有先验图/重定位地图。vkernel 在场景加载时把全场景点云
  （z 归一化、0.15m 体素降采样、不过 blind/FOV/遮挡）一次性 latch 发布到
  `/prior_map/cloud`；launch `map_mode:=prior` 时 PCT 的
  `explored_areas` 改订阅该话题，在全场景层析图上规划全局路径，
  adapter 用确定性 lookahead 截取把全局路径逐段喂给 SCAN——
  AABB 尺寸的接力由 lookahead 窗口自然形成，不需要学习模型切航点。

语义/编排层只影响候选偏好与恢复选择，几何层（PCT/SCAN 占据判定）始终有权否决。

## 地形安全约束（legged-terrain-state）

底层运动约束采用 vendored 的 [legged-terrain-state](https://github.com/zxinyu-robot/legged-terrain-state)（`third_party/legged_terrain_state`，MIT）：

```
vkernel / 真机感知 ──/terrain/observation──▶ terrain_safety_node ──/terrain/decision──▶ cmd_vel_forwarder
   (TerrainObservation)                       (LTS TerrainAssessor)   (TerrainDecision)     (施加 safety_mode)
```

- 感知只产出观测（支撑高度/法向/粗糙度/支撑点数），不指挥运动；评估节点按坡度、粗糙度、支撑置信度给出
  `risk`（Nominal/Caution/Critical）与 `safety_mode`（Normal/ReduceSpeed/HoldPosition），故障位与 LTS `FaultFlag` 一致。
- forwarder：`ReduceSpeed` 速度乘 `reduce_speed_factor`（默认 0.4），`HoldPosition` 置零；
  从未收到决策视为无地形感知（放行），收到后超过 `terrain_decision_timeout_ms`（默认 1500）未更新则 fail-safe 停车。
- 评估节点观测超时（`obs_timeout_ms`，默认 1000）主动发 `Stale + Critical + HoldPosition`。
- vkernel 观测：高程图邻域（`terrain_obs_radius`，默认 0.6 m）平面拟合得法向，残差 RMS 作粗糙度，
  支撑层 ±step_z/2 内场景点计数作支撑点（`terrain_expected_points` 为期望值）；无高程图时给平地名义观测。
- `terrain_safety_enabled:=false` 可在 forwarder 侧关闭该约束。
