# MotionSLAM_ws

Unitree Go2 端侧自主导航栈（ROS 2 Humble）：Super-LIO → PCT 全局规划 → BT 任务编排 → SCAN 局部规划 → forwarder → Sport 运动执行。

## 云端跨楼层闭环演示

虚拟设备内核（vkernel）多楼层仿真：一楼 → 开敞坡道 → 二楼 → 原路返回，
PCT→BT→SCAN→forwarder 全链路闭环，11/11 子目标 `mission_succeeded`。

<video src="docs/media/cross_floor_demo.mp4" controls width="880"></video>

[直接下载/打开视频](docs/media/cross_floor_demo.mp4) ｜
[云端闭环跑法与原理](docs/云端闭环_vkernel.md)

- 场景：`src/motionslam_bringup/config/vkernel_multifloor.yaml`（高度场：walls z0 + floors + ramps）
- 渲染：`src/motionslam_bringup/scripts/cloud_video_render_3d.py`（`/lio/cloud_world` 点云 + `/lio/robo/odom`，Foxglove 风格高度着色）

## 分支

- `main`：稳定基线
- `dev`：开发主线（其余分支已收口）
