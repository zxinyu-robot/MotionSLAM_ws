# MotionSLAM 未完成工作梳理

> 分支 `roadmap`。按"必须上真机验证 / 目标架构推进 / 工程质量 / 云端仿真增强"
> 四组梳理，标注依赖与建议优先级（P0 阻塞项 → P2 可延后）。

## A. 真机验证 backlog（P0，云端结论都需真机复核）

- [ ] **边端闭环**：edge uplink / 边端协同链路未在真机验证（云端 `with_edge_uplink:=false`）
- [ ] **返航 return_home**：玻璃 trap 已移除并改名 `slow_mode`，真机返航未复测
- [ ] **动态障碍**：virtual_obstacle_merge 管道保留，动态/行人场景未验收
- [ ] **长时耐久**：目前只验证过 1m 短程（偏差 0.43m）与云端多楼层仿真
- [ ] **跨楼层高度过滤**：真机 `obstacle_height_max=1.50`（相对地面）会滤掉
  楼上墙体——需改成相对机器人或分层地图后才可部署跨楼层任务
- [ ] **SCAN pin 修复**：`d62de08` 是 Jetson 本地提交不在上游，需推到
  `wuyi2121/SCAN-Planner` 或更新 THIRD_PARTY.md，否则换机不可复现
- [ ] **minSnapTraj 共线奇异复测**：云端已修（`decimate_collinear_xy` +
  前导点丢弃），真机复现同类现象需确认修复效果

## B. 目标架构（空间状态→任务意图→安全动作→新状态）

- [ ] **task_runtime 接管编排**：目前是脚手架 + Thin Harness 意图入口已验证
  （intent → task_terminal），BT 的 1832 行单文件状态机应逐步迁到
  task_runtime 的生命周期模型，BT 退化为"导航技能"之一
- [ ] **符号空间正式化**：任务意图 schema、生命周期状态机、异常回退路径
  需要落到 task_runtime + Thin Harness 契约文档
- [ ] **motionslam_ipc 共享内存点云**：无任何调用方，Super-LIO 写入改动
  不在 patches——要么补全要么删除，先决策
- [ ] **三空间边界守护**：世界空间只提交意图、度量空间独占实时执行的
  约束目前靠约定，需在接口层固化（意图校验已有一版 `task_intent.py`）

## C. 工程质量

- [ ] **子模块注册**：`.gitmodules` 列了 5 个组件但无 gitlink，
  `git submodule update --init` 拉不到东西——注册 gitlink 或改纯文档流程
- [ ] **CI**：task_runtime CTest + bringup pytest（除 test_semantic_mission.py）
  在普通 Ubuntu 即可跑，接 GitHub Actions 最小 CI
- [ ] **死代码清理**：4 个无调用方的头文件 + `TrajTracker`（仅单测引用）
- [ ] **BT 状态机拆分**：`demo_scan_bt_orchestrator.py` 单文件状态机过大
- [ ] **Dockerfile.cloud 补 ffmpeg/matplotlib**：云端渲染依赖目前是手工装的

## D. 云端仿真增强

- [x] vkernel 多楼层高度场 + 跨楼层闭环（11/11 mission_succeeded）
- [x] 双视口渲染（实时点云 + 真实 occupancy_inflate + 轨迹带）
- [ ] **占据图滑动窗口对齐**：渲染侧目前按 first-seen 永久累积体素，
  真机 occupancy 是滑动窗口（会遗忘），视频语义可再对齐
- [ ] **多楼层真机参数回放**：把真机 bag 灌进 vkernel 回放验证一致性
- [ ] **边端链路云端化**：edge uplink 在云端也跑起来（当前关掉）

## 待决策项

1. PR #6（dev→main）合并——需人工批准
2. motionslam_ipc 补全还是删除
3. SCAN `obstacle_height_*` 是否改成相对机器人（影响真机参数基线）
