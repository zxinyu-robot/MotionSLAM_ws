# Demo 真机 Runbook

> 无先验建图 · **默认 PCT C++ 常驻** · SCAN 连续局部 · BT 编排  
> 进度与卡点：[项目规划_Demo1三步.md](../规划/项目规划_Demo1三步.md) · 验收事实：[测试记录.md](../测试验收/测试记录.md)

## 启动

```bash
cd ~/MotionSLAM_ws
./scripts/motionslam nav start    # 起容器 + 停官方 SLAM + 起栈
# Foxglove: ws://${GO2_IP}:8765  Fixed Frame=world
# 异常: ./scripts/motionslam nav watch   （实时事件: nav watch -f）
./scripts/motionslam nav stop
```

`nav start` 默认：`WITH_PCT_PLANNER=true`、`WITH_GLASS_AWARE=true`、`WITH_FOXGLOVE=true`、`AUTOSTART=true`、`DEMO_PROFILE=demo_short`。  
边端 uplink / semantic_objnav / BEV 默认关。没有 `nav start-lean`。

## 默认主链

| 项 | 日常默认（PCT 开） | 对照（`WITH_PCT_PLANNER=false`） |
|----|-------------------|----------------------------------|
| 定位 | Super-LIO mapping，任意开机点 | 同左 |
| 全局 | `pct_resident_planner` → `/global_path` | 无；BT 直写 SCAN goal |
| BT → 执行 | `/initial_path`（加密后的 PCT 或返航折线） | `move_base_simple/goal` |
| SCAN | `fsm.navi_mode=3` 路径跟踪 | `navi_mode=1` |
| 玻璃 | `virtual_obstacle_merge` + `glass_suspect_layer` | 可 `WITH_GLASS_AWARE=false` 关掉 |
| Context | `demo_navigation_context` | 同左 |

```text
BT /goal_pose → PCT /global_path → BT /initial_path
  → SCAN B-spline → closed_loop → /motion/command → forwarder → Sport
```

08-19 短程验收关了 Foxglove 和玻璃，用 `acceptance_short_waypoints.yaml` 走通 1.0 m 到达。日常开栈与那次开关不完全相同，但数据流相同。

## 观测

BT autostart：`mission_wait_cloud` → `cloud_ready` → `mission_started` → `subgoal_dispatched` →（PCT）`pct_path_received` → SCAN 出 `/planning/bspline`。

```bash
ros2 topic echo /demo/mission/event std_msgs/msg/String
ros2 topic echo /demo/lifecycle/state std_msgs/msg/String
ros2 topic echo /navigation_context/ready std_msgs/msg/Bool
ros2 topic echo /global_path nav_msgs/msg/Path
ros2 topic echo /initial_path nav_msgs/msg/Path
ros2 topic echo /planning/bspline scan_planner_msgs/msg/Bspline
```

`nav watch` 会从最近 `logs/demo_scan_nav_*.log` 抽 `pct_path_received` / `subgoal_dispatched` / `plan_fail` / `glass_trap` 等。代码里没有 `pct_nav_started` 事件。

手动启动（等 Foxglove 看点云稳定后再动）：

```bash
AUTOSTART=false ./scripts/nav/start_demo_scan_nav.sh
# 稳定后
ros2 topic pub --once /demo/mission/start std_msgs/msg/String "{data: 'go'}"
```

Foxglove Fixed Frame: `world`（`demo_medium` / `demo_reloc` 用 `map` 时改 Fixed Frame 与 `grid_map.frame_id` 一致）

### Foxglove：占据栅格 + 规划轨迹

连接 `ws://<NX_IP>:8765`，用 **3D** 面板。日常栈 `foxglove_profile:=demo_scan`，`with_legacy_bridge:=false`（只有 8765）。

| 层 | 话题 | 显示 | 设置 |
|----|------|------|------|
| **局部 SCAN** | `/grid_map/occupancy_inflate` | 局部占据/膨胀体素 | Shape=**Cube**, Size=**0.05**, Color by **Z** |
| | **`/planning/trajectory_path`** | **实时 B-spline（推荐）** | 3D → **Path** |
| | `/optimal_list` | C++ 优化轨迹 | Markers |
| | `/initial_path` | BT 交给 SCAN 的参考路径 | Path |
| **全局 PCT** | `/global_path` | C++ A* 粗路径 | 3D → Path |
| | `/tomogram` | 仅 Python PCT 对照时有 | 可选 |
| **位姿** | `/lio/pose` 或 `/lio/robot_marker` | 狗位姿 | robot_viz 自动发布 |
| **编排** | `/demo/mission/event` | BT 任务事件 | Raw Messages |

更多话题：`foxglove_profile:=full`。`grid_map.debug_topics:=true` 已在 `demo_scan_planner.yaml` 默认开启。详见 [Foxglove代价图可视化.md](../开发参考/Foxglove代价图可视化.md)。

## Lifecycle

```bash
ros2 topic pub --once /demo/lifecycle/transition std_msgs/msg/String "{data: 'configure'}"
ros2 topic pub --once /demo/lifecycle/transition std_msgs/msg/String "{data: 'activate'}"
ros2 topic pub --once /demo/mission/start std_msgs/msg/String "{data: 'go'}"
ros2 topic pub --once /demo/mission/cancel std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub --once /demo/lifecycle/transition std_msgs/msg/String "{data: 'deactivate'}"
```

返航（面包屑倒序 `/initial_path`，不调 PCT）：

```bash
ros2 topic pub --once /demo/mission/return_home std_msgs/msg/String "{data: 'go'}"
```

## 干跑 / 真机 / 可选开关

```bash
# 干跑：不起 closed_loop / forwarder
./scripts/nav/start_demo_scan_nav.sh with_forwarder:=false

# 08-19 短程验收复现（关 Foxglove / 玻璃）
WITH_FOXGLOVE=false WITH_GLASS_AWARE=false AUTOSTART=true \
WAYPOINTS_FILE=/ws/src/motionslam_bringup/config/acceptance_short_waypoints.yaml \
./scripts/nav/start_demo_scan_nav.sh

# 边端控制面（默认关）
WITH_EDGE_UPLINK=true WITH_SEMANTIC_OBJNAV=true ./scripts/nav/start_demo_scan_nav.sh

# Python PCT 对照（与 C++ /build_tomogram 冲突，仅调试）
./scripts/nav/start_demo_scan_nav.sh with_pct_python:=true
```

不要把「直接 `pub /goal_pose`」当成日常验收：BT 按 stamp 丢弃对不上的 `/global_path`。

## 停止

```bash
./scripts/motionslam nav stop
# 或
./scripts/nav/stop_nav.sh
```

开栈 = API 模式（断手柄可走）；停栈 = 恢复手柄。

## 验收要点

- [ ] 无 `map_reloc.pcd` / 无 `reloc_spot` 可启动（mapping 模式）
- [ ] `/navigation_context/ready=true`
- [ ] PCT 在线：日志出现 `PCT online tomogram voxels=…`；`/global_path` stamp 与 `/goal_pose` 匹配
- [ ] BT 事件含 `subgoal_dispatched`、`pct_path_received`；SCAN 出 `/planning/bspline`
- [ ] cancel / `nav stop` 后 `/motion/command` 归零
- [ ] 板载短程到达已于 2026-08-19 通过（见测试记录 BOARD-CL）；边端语义链、真机返航未关单
