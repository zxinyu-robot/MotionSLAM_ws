# F 系列 · 前段正确性验收（开发测试说明）

> **不是当前 PCT 主链验收。** 主链事实见 [测试记录.md](../测试验收/测试记录.md)。  
> F3/F4 依赖的 `ego_nav*.launch.py` 已从 dev 移除；当前开栈用 `demo_scan_stack.launch.py` / `./scripts/motionslam nav start`。  
> 完整 ego/Nav2 F 系列见 `feature/hybrid-baseline` 与 `scripts/legacy_nav2/`。  
> 一键脚本：[`scripts/verify/verify_f_series.sh`](../../scripts/verify/verify_f_series.sh)  
> 关联：F1 详见 [T2.5_odom_robo外参标定.md](T2.5_odom_robo外参标定.md)

---

## 1. 总览

| 编号 | 验证点 | 通过标准 | 脚本 |
|------|--------|----------|------|
| F1 | LIO 位姿 pitch | `/lio/robo/odom` vs `/utlidar/robot_odom` pitch 差 < 3° | `calibrate_odom_robo.py --report-only` |
| F2 | 定位漂移 | 静止 60s 漂移 < 0.05 m；直线 5 m 回测误差 < 0.15 m | `verify_f2_drift.py` |
| F3 | 点云语义 | `/lio/cloud_world` 障碍高度带内有点 | `verify_f3_demo_cloud.py` |
| F4 | 规划可行 | 空场发 goal，timeout 内收到 `/planning/bspline` | `verify_f4_demo_scan.py` |

---

## 2. 前置 launch

| 项 | 最低要求 launch | 说明 |
|----|-----------------|------|
| F1 | `mapping.launch.py` / `slam_viz` / `demo_scan_stack` | Super-LIO + Unitree odom |
| F2 | 同上 | 仅需 `/lio/odom` 或 `/lio/robo/odom` |
| F3 | `demo_scan_stack`（可 `with_forwarder:=false`） | 需 `/lio/cloud_world` + `/grid_map/occupancy_inflate` |
| F4 | 同上 | 需 `scan_planner_node` 出 `/planning/bspline`；**不是** `ego_planner_node` |

```bash
# 终端1: 干跑导航 (F3/F4 需要)
./scripts/nav/start_demo_scan_nav.sh with_forwarder:=false

# 终端2
source /ws/scripts/dev/source_ws_env.sh

./scripts/motionslam verify f-series
./scripts/motionslam verify f-series --full          # 静止 60s
./scripts/motionslam verify f-series --line          # 遥控直线 5m 再回起点
```

---

## 3. 参考文件

### 3.1 脚本

| 文件 | 作用 |
|------|------|
| [`scripts/verify_f_series.sh`](../../scripts/verify_f_series.sh) | F1–F4 编排；`--full` / `--line` |
| [`scripts/calibrate_odom_robo.py`](../../scripts/calibrate_odom_robo.py) | F1 |
| [`scripts/verify_f2_drift.py`](../../scripts/verify_f2_drift.py) | F2 静止 / 直线 |
| [`scripts/verify_f3_costmap.py`](../../scripts/verify_f3_costmap.py) | F3 点云–代价图匹配 |
| [`scripts/verify_f4_planning.py`](../../scripts/verify_f4_planning.py) | F4 发 goal 验 bspline |
| [`scripts/e2e_verify.sh`](../../scripts/e2e_verify.sh) | 全栈冒烟 (含 F 子集 + H) |

### 3.2 配置与 launch

| 文件 | 关联项 |
|------|--------|
| [`config/super_lio_mid360.yaml`](../../src/motionslam_bringup/config/super_lio_mid360.yaml) | F1 `odom_robo`; F2 `/lio/odom` |
| [`config/demo_scan_planner.yaml`](../../src/motionslam_bringup/config/demo_scan_planner.yaml) | F3/F4 SCAN grid_map / bspline |
| [`launch/demo_scan_stack.launch.py`](../../src/motionslam_bringup/launch/demo_scan_stack.launch.py) | 当前 F3/F4 数据流 |

`ego_nav*.launch.py` / `ego_grid_map.yaml` / `ego-planner-swarm` 仅历史对照（`feature/hybrid-baseline`）。

### 3.3 源码（语义）

| 文件 | 说明 |
|------|------|
| Super-LIO `ROSWrapper.cpp` | `/lio/odom`, `/lio/robo/odom`, `/lio/cloud_world` |
| SCAN `grid_map` | `/grid_map/occupancy_inflate` |
| SCAN `scan_planner_node` | `/planning/bspline`；PCT 开时跟 `/initial_path` |

### 3.4 ROS 话题

| 话题 | F 项 |
|------|------|
| `/lio/odom` | F2 |
| `/lio/robo/odom` | F1 |
| `/utlidar/robot_odom` | F1 参考 |
| `/lio/cloud_world` | F3 输入 |
| `/grid_map/occupancy_inflate` | F3 输出 |
| `/initial_path` 或 `move_base_simple/goal` | F4 输入（视是否开 PCT） |
| `/planning/bspline` | F4 输出 |

---

## 4. 分项说明

### F1 · LIO pitch（已通过 2026-07-13）

`odom_robo pitch = 11.8°`，pitch 差 **0.43°**。见 [T2.5 文档](T2.5_odom_robo外参标定.md)。

### F2 · 定位漂移

```bash
# 静止 60s (狗不动)
python3 scripts/verify_f2_drift.py --static --duration 60

# 直线回测 (遥控去程 + 回起点)
./scripts/verify_f_series.sh --line --target-m 1   # 试跑 1m, 回测限 ~0.05m
./scripts/verify_f_series.sh --line --target-m 3   # 试跑 3m, 回测限 ~0.09m
./scripts/verify_f_series.sh --line --target-m 5   # 正式 5m, 回测限 0.15m
```

### F3 · 代价图语义

- 在 **有障碍** 环境测试（空场会 SKIP）
- 默认障碍高度带 world z ∈ [-0.20, 0.55]（对齐 `ego_grid_map.yaml` ground_height）
- 默认 recall ≥ 0.45、precision ≥ 0.35

```bash
python3 scripts/verify_f3_costmap.py --once
```

### F4 · 规划可行

- 当前方 **2 m** 发 goal（沿 odom yaw）
- **goal z 须 ≥ -0.1**（ego FSM 会丢弃更低 z；脚本用 `max(odom.z, 0)`）
- **空场** 测试；持续 `optimization failed` 则 FAIL
- 会实际触发一次 replan（干跑安全）

```bash
python3 scripts/verify_f4_planning.py --once --distance 2.0
```

---

## 5. 验收记录模板

完成 `--full` + `--line` 后填入 [测试记录.md](../测试验收/测试记录.md)：

| 编号 | 状态 | 日期 | 备注 |
|------|------|------|------|
| F1 | 通过 | 2026-07-13 | pitch 差 0.44° |
| F2 | 部分通过 | 2026-07-13 | 60s 漂移 0.0097m; 3m 直线回测 0.087m; 5m 待测 |
| F3 | 通过 | 2026-07-13 | recall 1.00 / precision 0.66 |
| F4 | 通过 | 2026-07-13 | bspline pos_pts=11 |

---

## 6. 变更历史

| 日期 | 变更 |
|------|------|
| 2026-07-13 | 新增 F2–F4 脚本与 `verify_f_series.sh` |
