# MotionSLAM_ws

Unitree Go2 + Mid-360 端侧自主导航（ROS 2 Humble）：解决**足式机器人在多楼层结构化场景下的自主定位与导航**。链路：Super-LIO 定位 → PCT 全局规划 → BT 任务编排 → SCAN 局部规划 → forwarder → Sport 运动执行。

## 贡献

- **三空间状态化数据流**：空间状态（世界空间）→ 任务意图（符号空间）→ 安全动作（度量空间）→ 新状态，闭环往复
- **云端可复现闭环**：vkernel 虚拟设备内核，无真机跑通全链路（多楼层 11/11 `mission_succeeded`）
- **任务意图入口**：Thin Harness + task_runtime，intent → task_terminal 验证通过

![数据状态流转时序图](docs/media/state_flow_seq.png)

## 真机部署规范

- 硬件：Jetson Orin NX + Unitree Go2 + Livox Mid-360；软件：ROS 2 Humble（Docker）
- 分支：`main` = 稳定基线（真机发布只走 main），`dev` = 开发主线
- 第三方依赖按 [THIRD_PARTY.md](THIRD_PARTY.md) 的 pin 手动拉取（`.gitmodules` 无 gitlink）
- 导航参数以 `src/motionslam_bringup/config/demo_scan_planner.yaml` 为真机基线；云端 overlay `demo_scan_planner_cloud.yaml` 仅放宽动力学与障碍 z 过滤，不得回灌真机
- 跨楼层部署前需评估 SCAN 障碍高度过滤（真机 `obstacle_height_max=1.50` 相对地面，建议改相对机器人/分层地图）
- 玻璃/透明障碍：仅保留类别无关的 `virtual_obstacle_merge` 管道，玻璃专用策略按规划文档已移除

## 云端闭环演示

![云端跨楼层闭环演示](docs/media/cross_floor_demo.webp)

[完整视频 mp4](docs/media/cross_floor_demo.mp4)

## 参考工作

- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)（`ros2` 分支）— 定位与建图
- [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)（`ros2-community` 分支）— 局部规划与轨迹控制
- [PCT_planner](https://github.com/VectorRobotics/PCT_planner) — 全局路径规划
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) — Go2 运动接口
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) — Mid-360 驱动
