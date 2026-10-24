# MotionSLAM_ws

Unitree Go2 端侧自主导航栈（ROS 2 Humble）：Super-LIO → PCT 全局规划 → BT 任务编排 → SCAN 局部规划 → forwarder → Sport 运动执行。

## 数据状态流转

基于状态的链路：世界空间产出空间状态，符号空间承载任务意图，度量空间执行安全动作，执行结果回写为新状态，闭环往复。

![数据状态流转时序图](docs/media/state_flow_seq.png)

## 云端跨楼层闭环演示

虚拟设备内核（vkernel）多楼层仿真：一楼 → 开敞坡道 → 二楼 → 原路返回，
PCT→BT→SCAN→forwarder 全链路闭环，11/11 子目标 `mission_succeeded`。

![云端跨楼层闭环演示](docs/media/cross_floor_demo.webp)

[下载完整视频 mp4](docs/media/cross_floor_demo.mp4)（左：实时点云 `/lio/cloud_world`；右：在线体素占据 `/grid_map/occupancy_inflate` + 轨迹优化）

- 场景：`src/motionslam_bringup/config/vkernel_multifloor.yaml`（高度场：walls z0 + floors + ramps）
- 渲染：`src/motionslam_bringup/scripts/cloud_video_render_3d.py`（`/lio/cloud_world` 点云 + `/lio/robo/odom`，Foxglove 风格高度着色）

## 分支

- `main`：稳定基线
- `dev`：开发主线（其余分支已收口）
