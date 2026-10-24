# MotionSLAM 未完成工作梳理

> 分支 `roadmap`。按"上真机 / 目标架构推进 / 工程质量 / 云端仿真增强"
> 四组梳理，标注依赖与建议优先级（P0 阻塞项 → P2 可延后）。

## A. 上真机（真机部署阶段事项，云端结论都需真机复核）

- [ ] **边端闭环**：edge uplink / 边端协同链路真机验证（云端 `WITH_EDGE=1`
  已可跑，真机未验）
- [ ] **返航 return_home**：玻璃 trap 已移除并改名 `slow_mode`，真机返航复测
- [ ] **动态障碍**：virtual_obstacle_merge 管道保留，动态/行人场景验收
- [ ] **长时耐久**：目前只有 1m 短程（偏差 0.43m）与云端多楼层仿真
- [ ] **跨楼层高度过滤**：真机 `obstacle_height_max=1.50`（相对地面）会滤掉
  楼上墙体——需改成相对机器人或分层地图后才可部署跨楼层任务
- [ ] **SCAN pin 修复**：`d62de08` 是 Jetson 本地提交不在上游，需推到
  `wuyi2121/SCAN-Planner` 或更新 THIRD_PARTY.md（gitlink 当前指到
  ros2-community 分支头 d0b921c，真机 pin 推上游后再换）
- [ ] **minSnapTraj 共线奇异复测**：云端已修（`decimate_collinear_xy` +
  前导点丢弃），真机复现同类现象需确认修复效果

## B. 目标架构（空间状态→任务意图→安全动作→新状态）

> 三空间约束语义：**世界空间**=空间状态（几何观测事实）；
> **符号空间**=语义价值信息约束（任务意图/决策在此空间）；
> **度量空间**=几何约束（安全动作边界，SCAN/执行层保证）。

> 原则：项目面向工程落地，编排/运行时/恢复机制的权威实现都在 **C++**
> （`runtime/task_runtime`、`src/motionslam_pipeline`）。Python 层
> （demo_scan_bt_orchestrator、thin_harness、task_runtime_adapter）是
> 过渡形态与验证工具，新能力不进 Python。
> 决策编排终态不是状态机：**决策模型 + ESN（回声状态网络）驱动**——
> task_runtime 承载生命周期/事件流，决策模型消费空间状态产出
> skill_sequence，ESN 提供时序上下文记忆。现有 BT/Python 状态机
> 只是当前可跑的过渡实现。

- [x] **符号空间契约**：`docs/符号空间契约.md`（TaskIntent schema +
  生命周期状态机 + 校验规则 + 事件流）
- [x] **task_runtime Python 适配层（过渡）**：`task_runtime_adapter.py` 与
  C++ TaskRuntime 同契约，仅用于云端验证入口链路；工程形态以 C++ 为准
- [x] **motionslam_ipc**：决策=移除。无任何调用方、Super-LIO 写入改动不在
  patches；包、依赖声明、唯一消费头文件均已删
- [ ] **决策模型 + ESN 决策层**（P0，新分支）：符号空间的任务决策由
  决策模型 + ESN 网络驱动——输入空间状态（占据图/位姿/任务上下文），
  输出 skill_sequence 给 task_runtime 调度；ESN 提供时序上下文记忆
  （卡死/漂移/恢复历史进状态）。不是把现有状态机搬成 C++。
- [ ] **状态机能力向决策模型移交**：Python 编排器的
  execute/return_home/recovery/slow_mode/stuck 检测语义，作为决策模型的
  安全兜底与训练/蒸馏素材保留；恢复机制（plan_fail_recovery/escalate/
  drift）进 C++ task_runtime + motionslam_pipeline
- [ ] **跨楼层自主探索闭环**：frontier 探索 + 恢复机制在 C++ 侧形成
  完整探索-导航-恢复循环（当前链路是 waypoints 驱动）
- [ ] **三空间边界守护固化**：世界空间只提交意图、度量空间独占实时执行
  的约束在接口层固化（意图校验已有一版，跨空间调用审计未做）

- [ ] **状态契约 + BT 控制逻辑**（五层边界见 `docs/符号空间契约.md`；图/文档已定，
  代码按下列优先级在 C++ 侧落地：task_runtime + motionslam_pipeline）
  - P0 状态反馈与执行结果反馈分离：状态层 → BT 黑板；控制层 ExecStatus → BT，
    BT 不得把状态更新当成任务成功
  - P0 速度指令与设备接口分离：控制层只出 vx/vy/ωz，限幅/看门狗/StopMove 归设备层
  - P0 全局路径 → 局部参考接口契约：`/initial_path` 携带 frame、plan_id、subgoal_idx
  - P1 中间子目标循环、终止条件、失败恢复（到达判定加速度稳定条件）
  - P1 PCT 可通行性表示（tomogram：坡度/台阶高度差/垂直障碍/足迹）显式化
  - P1 统一 frame_id / stamp / 轨迹有效期，`task_id→plan_id→traj_id` 版本链
  - P2 PlanResult / ExecStatus 字段落地（GoalRequest、failure_code、
    tracking_error、trajectory_valid），便于诊断与测试

## C. 工程质量

- [x] **子模块注册**：5 个 gitlink 已注册（Super-LIO/livox_ros_driver2/
  unitree_ros2/PCT-Planner/SCAN-Planner），`git submodule update --init` 可用
- [x] **CI**：`.github/workflows/ci.yml`——task_runtime ctest +
  bringup pytest（排除依赖 ROS 的 test_semantic_mission.py）
- [x] **死代码清理**：NavigationBtConditions/SharedMapBuffer/
  ShmCloudObstacleLayerCore/WaypointFsmCore 4 个头文件 + TrajTracker
  （头+测试）已删
- [x] **BT 状态机拆分（第一步）**：`demo_scan_bt_core.py` 抽出枚举/
  SyncParameterClient/Subgoal/MissionState/load_waypoints，编排器 1681→1529 行；
  剩余 ~1448 行节点类的状态机分段（execute/return_home/recovery）待继续拆
- [x] **Dockerfile.cloud 补 ffmpeg/matplotlib/fonts-noto-cjk**

## D. 云端仿真增强

- [x] vkernel 多楼层高度场 + 跨楼层闭环（11/11 mission_succeeded）
- [x] 双视口渲染（实时点云 + 真实 occupancy_inflate + 轨迹带）
- [x] **占据图滑动窗口**：渲染器 `--occ-window <秒>`，体素按 last_seen
  遗忘，对齐在线占据图语义（0=累积，保持现有视频行为）
- [x] **边端链路云端化**：`WITH_EDGE=1 scripts/dev/cloud_sim_multifloor.sh`
  自动起 mock 接收端 + `with_edge_uplink:=true edge_host:=127.0.0.1`
- [ ] **多楼层真机参数回放**：把真机 bag 灌进 vkernel 回放验证一致性
  （缺真机 bag 素材，阻塞）

## 待决策项

1. PR #6（dev→main）合并——需人工批准
2. ~~motionslam_ipc 补全还是删除~~ → 已定：删除（见 B）
3. SCAN `obstacle_height_*` 是否改成相对机器人（影响真机参数基线）
