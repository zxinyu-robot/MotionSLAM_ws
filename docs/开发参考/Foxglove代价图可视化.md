# Foxglove 局部代价图可视化（occupancy_inflate）

> 目标：在 Foxglove 中复现按高度着色的 3D 体素代价图。  
> 当前主链：`demo_scan_stack.launch.py` + SCAN `grid_map`（`demo_scan_planner.yaml`）。`ego_nav*.launch.py` 已从 dev 移除。

---

## 1. 数据链路

```text
mid360
  → Super-LIO (lio.sensor.blind 近距球滤波)
  → /lio/cloud_world + /lio/robo/odom
  → SCAN grid_map（玻璃开时点云为 /perception/scan_cloud_fused）
  → /grid_map/occupancy_inflate   ← Foxglove 3D 主话题
  → foxglove_bridge (:8765)
```

| 参数 | 文件 | 含义 |
|------|------|------|
| `lio.sensor.blind` | `super_lio_mid360.yaml` | mid360 近距盲区半径 (m) |
| `grid_map.local_update_range_*` | `demo_scan_planner.yaml` | 局部代价图更新范围 |
| `grid_map.resolution` | `demo_scan_planner.yaml` | 体素边长，Foxglove Cube size 应对齐（默认 0.05） |
| `grid_map.debug_topics` | `demo_scan_planner.yaml` | 为 true 才 advertise `/grid_map/*`（已默认开） |

---

## 2. 启动

```bash
# 日常：nav start 已带 Foxglove（WITH_FOXGLOVE=true）
./scripts/motionslam nav start

# 仅桥（全量话题）
ros2 launch motionslam_bringup viz.launch.py foxglove_profile:=full

# 干跑
./scripts/nav/start_demo_scan_nav.sh with_forwarder:=false
```

日常栈 `foxglove_profile:=demo_scan`，`with_legacy_bridge:=false`（只有 8765）。白名单见 `viz.launch.py` → `DEMO_SCAN_WHITELIST`（含 `/global_path`、`/initial_path`、`/planning/trajectory_path`）。C++ PCT **不**发 `/tomogram`。

冒烟：

```bash
ros2 topic hz /grid_map/occupancy_inflate
ros2 topic echo /grid_map/occupancy_inflate --once | head
```

NOTE: `publishMapInflate` 仅在有订阅者时发布；Foxglove 连上并勾选话题后才有 hz。

---



## 3. Foxglove 面板设置（对齐 APP）

1. 连接 `ws://${GO2_IP}:8765`（或当前 NX IP）
2. 添加 **3D** 面板（不要用 Map 面板看体素）
3. **Fixed Frame** = `world`（**不要用** `base_link`，否则代价图会随机身 pitch 看起来是斜的）
4. 3D 视角点 **Top** 或 Reset view，让 Z 轴朝下（鸟瞰）
5. 勾选 `/grid_map/occupancy_inflate`，建议：


| 设置          | 值                            | 说明                           |
| ----------- | ---------------------------- | ---------------------------- |
| Point shape | **Cube**                     | 必选；默认小点会显得很稀                 |
| Point size  | **0.05**                     | 与当前 `grid_map/resolution` 一致 |
| Color mode  | Color map / Rainbow by **Z** | 低青绿 → 高黄粉，接近 APP             |
| Decay time  | 0                            | 局部滚动图                        |


可选叠加：


| 话题                              | 用途                                            |
| ------------------------------- | --------------------------------------------- |
| `/lio/pose`                     | **推荐** 狗位姿 (PoseStamped, 比 Odometry 稳定)       |
| `/lio/robot_marker`             | 狗身 CUBE 模型 (近似 Go2 尺寸)                        |
| `/lio/path`                     | 历史轨迹线                                         |
| `/lio/odom`                     | 原始里程计 (Foxglove 对空 child_frame 支持差, 优先用 pose) |
| `/planning/trajectory_path` | **推荐** 实时 B-spline Path |
| `/initial_path` | BT 交给 SCAN 的参考路径 |
| `/global_path` | PCT C++ 粗路径 |
| `/optimal_list` / `/goal_point` | 优化轨迹 / 目标点 |


NOTE: `demo_scan_stack` 在 Foxglove 开启时自动起 `robot_viz` 与 `planning_trajectory_viz`。`robot_pose_viz` 默认 **level_orientation=true**（仅保留 yaw）。


---



## 4. 地图看起来是斜的？


| 现象          | 原因                         | 处理                                               |
| ----------- | -------------------------- | ------------------------------------------------ |
| 整块代价图相对屏幕倾斜 | Fixed Frame 误选 `base_link` | 改 **Fixed Frame =** `world`                      |
| 鸟瞰仍略歪       | 3D 视角未对齐 Z                 | 点 **Top** / Reset view                           |
| 狗身与地图夹角大    | LIO odom 带 pitch（雷达系）      | 已默认 `level_orientation`; 狗身用 `/lio/robot_marker` |


真机对比：Super-LIO `/lio/odom` 常有 ~10–15° pitch，Unitree `/utlidar/robot_odom` 接近 0°；**规划与代价图仍在** `world` **系**, 只是可视化位姿差异。导航链路勿改 odom，只调 Foxglove 显示。

---



## 5. 调参建议（真机）


| 现象            | 优先改                                 |
| ------------- | ----------------------------------- |
| 近处撞、图上狗身边一大圈空 | 减小 `lio.sensor.blind`（现 0.8，可试 0.6） |
| 机身/噪声过敏、走不动   | 增大 `blind`（可试 1.0）                  |
| 高障碍在 3D 里被削平  | 增大 `visualization_truncate_height`  |
| 代价图范围太小/太大    | 调 `demo_scan_planner.yaml` 的 `local_update_range_x/y`          |


---



## 6. 与 2D `/map` 的区别


| 话题                            | 用途                              |
| ----------------------------- | ------------------------------- |
| `/grid_map/occupancy_inflate` | 局部 3D 体素代价图（本页，APP 类）           |
| `/map`（`cloud_to_grid`）       | 全局/建图 2D OccupancyGrid，用 Map 面板 |


