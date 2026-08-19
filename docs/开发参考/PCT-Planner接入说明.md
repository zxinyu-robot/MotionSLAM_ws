# PCT 接入说明（当前实现）

> 日常主链用 C++ `pct_resident_planner`，**不**起 Python/Open3D PCT。  
> 进度见 [项目规划_Demo1三步.md](../规划/项目规划_Demo1三步.md)；操作见 [Demo阶段3D_runbook.md](../运维/Demo阶段3D_runbook.md)。

---

## 1. 两套实现，默认只用 C++

| 项 | 日常默认 | 可选对照（调试） |
|----|----------|------------------|
| 开关 | `with_pct_planner:=true`（`nav start` 默认） | `with_pct_python:=true` |
| 节点 | `motionslam_pipeline` / `pct_resident_planner`（ROS 名 `pct_planner`） | 上游 `pct_planner` launch |
| 热路径 | C++ 体素缓存 + 2D A* | Python / Open3D |
| `/build_tomogram` | C++ 常驻服务 | 与 C++ **同名冲突**，不要同时开 |

源码：`src/motionslam_pipeline/src/PctResidentPlanner.cpp`。  
Launch：`src/motionslam_bringup/launch/demo_scan_stack.launch.py`。

Python PCT 工作区（`./scripts/dev/build_pct_planner.sh` + `pct_planner_ws`）只在对照调试时需要。`source_ws_env.sh` 仍会尝试 source `pct_planner_ws`，缺失不阻塞 C++ 主链。

---

## 2. 话题与服务（C++）

Launch remap（`demo_short`、无 PGO）：

| 接口 | 方向 | 实际话题 |
|------|------|----------|
| 点云 | 入 | `explored_areas` ← `/lio/cloud_world` |
| 里程计 | 入 | `state_estimation` ← `/lio/robo/odom` |
| 目标 | 入 | `/goal_pose`（仅 BT 写） |
| 粗路径 | 出 | `/global_path`（`header.stamp` = 请求 stamp） |
| 预热/重建 | 服务 | `/build_tomogram`（`std_srvs/Trigger`） |

C++ 节点 **不** 发布 `/tomogram`。Foxglove 里该话题仅在 Python PCT 对照时有意义。

行为要点：

- 点云够 `cloud_min_points`（默认 800）后定时预热栅格。
- `/goal_pose` 触发 A*；路径 stamp 回写请求 stamp，供 BT 拒收过期路径。
- 起终点会 snap 到自由格；snap 超限则发空路径。
- **不** 写 `move_base_simple/goal`、**不** 写 `/cmd_vel`。

---

## 3. 与 BT / SCAN 的交接

PCT 开启时 launch 覆盖：

- BT：`pct_global_nav_enabled=true`，`scan_track_pct_path=true`，`pct_fallback_direct=false`
- SCAN：`fsm.navi_mode=3`

```text
BT 发 /goal_pose
  → C++ 回 /global_path（stamp 必须等于请求）
  → BT 拒空路径、单点路径、伪短路径
  → BT 加密折线后发 /initial_path
  → SCAN navi_mode=3 跟踪
```

BT **不再** 把 PCT 模式下的执行目标写到 `move_base_simple/goal`。  
`with_pct_planner:=false` 时才是 navi_mode=1 + `move_base_simple/goal`（历史 lean 对照，不是日常入口）。

返航（断网 / `glass_trap` / `/demo/mission/return_home`）走面包屑倒序 `/initial_path`，**不** 调 PCT A*。

---

## 4. 构建与开栈

```bash
# 主工作区（含 pct_resident_planner）
./scripts/motionslam build
source /ws/scripts/dev/source_ws_env.sh

# 仅 Python PCT 对照
./scripts/dev/build_pct_planner.sh
```

```bash
./scripts/motionslam nav start
# 等价：WITH_PCT_PLANNER=true ./scripts/nav/start_demo_scan_nav.sh
```

手动预热（通常不需要，节点会自己预热）：

```bash
ros2 service call /build_tomogram std_srvs/srv/Trigger
```

不要在未改 BT 的情况下直接 `pub /goal_pose` 当日常用法：BT 会按 stamp 丢弃对不上的路径。验收短程用 `WAYPOINTS_FILE=.../acceptance_short_waypoints.yaml`，见 [测试记录.md](../测试验收/测试记录.md) §4。
