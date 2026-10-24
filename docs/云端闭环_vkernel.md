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
