# MotionSLAM_ws

> **GitHub 私人备份仓**（脱敏快照）。狗端集成仓仍以 GitLab `origin` 为准；上游 submodule 与 patch 见 [THIRD_PARTY.md](THIRD_PARTY.md)。  
> 雷达配置：复制 `src/motionslam_bringup/config/MID360_config.example.json` → `MID360_config.json` 后填现场 IP。

Go2 自主避障导航自研栈：**Super-LIO + PCT C++ 常驻粗引导 + SCAN 连续局部 + BT 编排 + C++ pipeline**（Sport 转发、lifecycle context），
运行于 Humble Docker 容器（host 网络 + CycloneDDS 与 Go2 机身 DDS 域 0 原生互通）。

> **dev 日常主链**：`pct_resident_planner` → BT 写 `/initial_path` → SCAN `navi_mode=3` → closed_loop → Sport。  
> 进度只看 [项目规划_Demo1三步.md](docs/规划/项目规划_Demo1三步.md)；验收事实只看 [测试记录.md](docs/测试验收/测试记录.md)。

## 文档职责

| 文档 | 回答 | 入口 |
|------|------|------|
| **规划** | 整体目标、范围、阶段划分、工作包、门禁与进度 | [项目规划_Demo1三步.md](docs/规划/项目规划_Demo1三步.md) |
| **架构** | 三空间中间件 OS、与智源 RoboOS 对照、分层、部署、控制/数据面、模块边界（目标态，不当进度清单） | [架构_端侧系统架构.md](docs/架构/架构_端侧系统架构.md) |
| **接口规范** | 边端边界消息与端口契约 | [边端协同接口规范.md](docs/项目规范/边端协同接口规范.md) |
| **运维 / 测试** | 怎么启停、怎么验、干跑/真机证据 | [Demo阶段3D_runbook.md](docs/运维/Demo阶段3D_runbook.md) · [测试记录.md](docs/测试验收/测试记录.md) |
| **开发参考** | PCT、DDS、标定、工程附录、边侧 Qwen2.5-VL、私人备份切仓 | [PCT 接入](docs/开发参考/PCT-Planner接入说明.md) · [工程附录](docs/开发参考/工程开发附录.md) · [Qwen2.5-VL](docs/开发参考/边侧Qwen2.5-VL接入说明.md) · [DDS](docs/开发参考/DDS通信性能优化参考.md) · [私人 GitHub 备份清单](docs/开发参考/私人GitHub备份清单.md) |

临时草稿只进 [`docs/_inbox/`](docs/_inbox/)（非权威，AI 默认不索引）。

## 快速开始

```bash
# 1. 构建镜像（首次，联网）
docker build -t motionslam:humble docker/

# 2. 释放 mid360（停用宇树 unitree_slam 栈）
./scripts/motionslam stop-slam

# 3. 进入容器
./docker/run_container.sh

# 4. 容器内编译主工作区 + SCAN 独立工作区
./scripts/motionslam build
cd scan_planner_ws && colcon build --symlink-install && cd ..
source /ws/scripts/dev/source_ws_env.sh

# 5. 互通冒烟测试
ros2 topic list | grep rt/
```

日常开栈不依赖 Python/Open3D PCT。`./scripts/dev/build_pct_planner.sh` 只在 `with_pct_python:=true` 对照调试时需要。

恢复官方 SLAM: `/unitree/module/unitree_slam/bin/unitree_slam eth0`

## 目录

| 路径 | 说明 |
|------|------|
| `docker/` | Humble 容器环境 |
| `src/Super-LIO` | LIO（子模块） |
| `src/livox_ros_driver2` | mid360 驱动（子模块） |
| `src/unitree_ros2` | 宇树 ROS2 消息（子模块） |
| `src/SCAN-Planner` | SCAN 局部规划（`scan_planner_node` + `closed_loop_controller`） |
| `scan_planner_ws/` | SCAN 独立 colcon 工作区 |
| `src/PCT-Planner` | 上游 Python/Open3D PCT（可选对照；日常不启） |
| `pct_planner_ws/` | 上游 PCT 独立 colcon 工作区（仅 `with_pct_python`） |
| `src/motionslam_ipc` | LIO 点云 / 地图 **POSIX SHM** 读写 |
| `src/motionslam_pipeline` | `pct_resident_planner`、`cmd_vel_forwarder`、`virtual_obstacle_merge`、`bev_glass_layer`、lifecycle |
| `src/motionslam_bringup` | Demo launch、BT 编排、参数 |
| `scripts/` | `motionslam` CLI · nav/ map/ verify/ dev/ ops/ |

## 建图

```bash
ros2 launch motionslam_bringup mapping.launch.py
# 或
ros2 launch motionslam_bringup slam_viz.launch.py
# 宿主机
./scripts/motionslam map start
```

## 自主导航 — 主链

### 默认数据流（`./scripts/motionslam nav start`）

| 层 | 职责 | 默认实现 |
|----|------|----------|
| **L0 Super-LIO** | mapping 模式 | `/lio/robo/odom` + `/lio/cloud_world` |
| **L4 PCT** | 开机预热体素栅格 + 2D A* | `pct_resident_planner`（节点名 `pct_planner`）→ `/global_path` |
| **L3 BT** | 唯一执行写口、stamp 校验、返航/恢复 | `demo_scan_bt_orchestrator`：收 `/global_path`，写 `/initial_path` |
| **L2 SCAN** | 路径跟踪 + 局部 B-spline | `fsm.navi_mode=3` 跟踪 `/initial_path` |
| **L1 forwarder** | 限幅 + Sport 下发 | `closed_loop_controller` → `/motion/command` → `cmd_vel_forwarder` |

```text
waypoints / semantic goal
  → BT 发 /goal_pose
  → pct_resident_planner 回 /global_path（stamp 必须匹配）
  → BT 加密后发 /initial_path
  → SCAN navi_mode=3 → closed_loop → /motion/command → forwarder → Sport
```

PCT 开时 **不** 把执行目标写到 `move_base_simple/goal`。`pct_fallback_direct` 默认 false，空路径不会直线抢跑。

`nav start` 默认：`WITH_PCT_PLANNER=true`、`WITH_GLASS_AWARE=true`、`WITH_FOXGLOVE=true`、`AUTOSTART=true`、`DEMO_PROFILE=demo_short`。边端 uplink / semantic_objnav / TensorRT BEV **默认关**。不存在 `nav start-lean`。

### 日常命令

```bash
./scripts/motionslam nav start    # 起容器 + 停官方 SLAM + 起栈
./scripts/motionslam nav watch    # 异常日志
./scripts/motionslam nav stop
```

详见 [Demo阶段3D_runbook.md](docs/运维/Demo阶段3D_runbook.md) · [PCT-Planner接入说明.md](docs/开发参考/PCT-Planner接入说明.md)。

### Foxglove

`ws://${GO2_IP}:8765`；Fixed Frame=`world`；局部 `/planning/trajectory_path`、全局 `/global_path`。日常栈只开 8765（`with_legacy_bridge:=false`）。

## 硬件前提

- mid360 在线；自研栈前执行 `./scripts/motionslam stop-slam`
