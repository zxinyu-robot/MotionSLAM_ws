# GO2 导航主路径说明

> 更新：2026-08-19 · **dev 日常 = PCT C++ 常驻 + SCAN 局部 + BT 编排**  
> 进度只看 [项目规划_Demo1三步.md](../规划/项目规划_Demo1三步.md)；本文是命令入口。

## 主链

| 组件 | 职责 |
|------|------|
| **Super-LIO mapping** | 任意点开机，`/lio/robo/odom` + `/lio/cloud_world` |
| **pct_resident_planner** | 预热体素栅格、2D A*、`/global_path`（节点名 `pct_planner`） |
| **BT 编排** | `demo_scan_bt_orchestrator`：校验 stamp、写 `/initial_path`、返航/恢复 |
| **SCAN 连续局部** | `scan_planner_node` `navi_mode=3` + `closed_loop_controller` |
| **Context** | `demo_navigation_context` |
| **forwarder** | 限幅 + Sport 下发 |

```text
BT /goal_pose → PCT /global_path → BT /initial_path
  → SCAN → closed_loop → /motion/command → forwarder → Sport
```

**代码锚点**

| 项 | 路径 |
|----|------|
| 入口 launch | `src/motionslam_bringup/launch/demo_scan_stack.launch.py` |
| 宿主机入口 | `./scripts/motionslam nav start` |
| PCT 日常 | `src/motionslam_pipeline/src/PctResidentPlanner.cpp` |
| PCT 对照 | `with_pct_python:=true` → 上游 `pct_planner.launch.py`（勿与 C++ 同开） |
| 参数 | `config/demo_scan_planner.yaml` |
| 短程验收航点 | `config/acceptance_short_waypoints.yaml` |
| Runbook | [Demo阶段3D_runbook.md](Demo阶段3D_runbook.md) |
| PCT how-to | [PCT-Planner接入说明.md](../开发参考/PCT-Planner接入说明.md) |

---

## 编译（容器内）

```bash
cd /ws
./scripts/motionslam build
source /ws/scripts/dev/source_ws_env.sh
```

SCAN overlay 需要已编译 `scan_planner_ws`。Python PCT 工作区不是日常开栈前提。

---

## 日常三条命令

```bash
cd ~/MotionSLAM_ws
./scripts/motionslam nav start    # 起容器 + 停官方 SLAM + 起栈
./scripts/motionslam nav watch    # 异常才看；实时事件加 -f
./scripts/motionslam nav stop
```

Foxglove：`ws://${GO2_IP}:8765`，Fixed Frame=`world`。`AUTOSTART=true` 时不必再 `pub /demo/mission/start`；`AUTOSTART=false` 时等点云稳定后再发。

---

## 控制权

- **开栈** = API 模式（断手柄可走）
- **停栈** = 恢复手柄（`./scripts/motionslam nav stop`）

---

## 相关文档

| 文档 | 用途 |
|------|------|
| [Demo阶段3D_runbook.md](Demo阶段3D_runbook.md) | Demo 真机操作 |
| [架构_端侧系统架构.md](../架构/架构_端侧系统架构.md) | 目标分层（不是已完成清单） |
| [PCT-Planner接入说明.md](../开发参考/PCT-Planner接入说明.md) | C++ / Python PCT 边界 |
| [项目规划_Demo1三步.md](../规划/项目规划_Demo1三步.md) | Demo1 执行顺序与验收 |
| [测试记录.md](../测试验收/测试记录.md) | 已发生的验收事实 |
