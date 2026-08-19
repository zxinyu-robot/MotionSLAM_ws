# Go2 专利交底 — 架构与差异原材料

> **用途**：C 族交底书附图 3–6 张、差异表、实施例文字  
> **边界**：「已验证」分两层——历史 lean（2026-08-07）与 **板载 PCT 短程闭环（2026-08-19）**。边端语义闭环、返航、Token 标「未关单」，勿写成已有。

---

## A. 系统结构图原材料

### 传感器与本体接口

| 组件 | 角色 |
|------|------|
| Livox Mid-360 | `/livox/lidar`、`/livox/imu` |
| Super-LIO mapping | 里程计、世界系点云 |
| PCT-Planner | C++ `pct_resident_planner` 常驻体素栅格 + 2D A*、`/global_path` |
| SCAN-Planner | 局部 B-spline、动态避障 |
| Unitree Sport API | 速度/步态执行（经 forwarder） |
| 遥控器 | 人工接管 / 急停 |

### 软件节点链（边端 NX，Docker Humble）

```text
livox_ros_driver2
    → Super-LIO mapping
    → pct_resident_planner (/goal_pose → /global_path)
    → demo_scan_bt_orchestrator（写 /initial_path）
    → SCAN-Planner navi_mode=3 + closed_loop
    → cmd_vel_forwarder（限幅 / Sport）
    → Sport API
```

**安全**：`estop_go2`、遥控器、`stop_nav.sh` 停栈 + 零速。

**容器边界**：

- 宿主机：Foxglove、部分调试、网络
- Docker `motionslam:humble`：ROS2 Humble 栈、Super-LIO、PCT、SCAN、脚本验收

### 建议附图

1. 总体框图：传感器 → LIO → PCT → BT → SCAN → forwarder → 狗
2. 容器边界图：宿主机 vs Docker
3. 安全降级：estop / stop_nav 触发点

---

## B. 当前真实数据流（按话题）

| 话题 / 路径 | 发布者 | 消费者 | 说明 |
|-------------|--------|--------|------|
| `/livox/lidar` | livox driver | Super-LIO | 点云输入 |
| `/livox/imu` | livox driver | Super-LIO | IMU |
| `/lio/robo/odom` | Super-LIO | BT、SCAN、verify | 定位/里程计 |
| `/lio/cloud_world` | Super-LIO | PCT、SCAN | 世界系点云 |
| `/global_path` | pct_resident_planner | BT | stamp 必须匹配 /goal_pose |
| `/initial_path` | BT | SCAN navi_mode=3 | 执行参考（PCT 折线或返航） |
| `/planning/bspline` | SCAN-Planner | closed_loop | 局部轨迹 |
| `/motion/command` | closed_loop | forwarder | 导航速度指令 |
| Sport | unitree 接口 | 本体 | 经 SDK/forwarder |

**TF 主链**：`world → base_link`（mapping 模式）；在线 PGO 可选 `map → world`。

---

## C. 与「点云上云 / VLA 直控 cmd_vel」的差异（工程事实）

### 1) 为何不让云端模型直接下发 `/cmd_vel`

- **安全**：本地 PCT/SCAN + forwarder 限幅 + estop 为最后防线；云端直控绕过 BT lifecycle 与急停语义。
- **延迟与断网**：VLN 推理在边端 PC；狗端必须 **断网可本地停车**。
- **地图与定位**：Super-LIO 与 `context_generation` 绑定在本机；点云全量上云带宽与同步成本高。
- **可复现**：当前验收以测试记录 BOARD-CL + 事件链，需固定栈与 git 工作区状态。

### 2) 地图如何版本化

| 资产 | 用途 |
|------|------|
| 在线 OctVox / 局部地图 | SCAN 实时规划 |
| PCT tomogram | on-demand 全局可行通行区 |
| SC-PGO session | 离线 `map_reloc.pcd` |
| subgraph snapshot | 边端语义 uplink |

版本标识：**git HEAD + `context_generation` / session_id**。

### 3) 失败如何区分

| 类型 | 可观测信号 |
|------|------------|
| PCT 全局失败 | tomogram 构建失败、无 `/global_path` |
| SCAN 局部失败 | emergency bspline、PlanFailHold |
| 控制失败 | 跟踪误差过大、行走距离≈0 |
| 急停 | `estop_go2` / verify 位移阈值 |

### 4) 断网时哪些能力仍本地有效

| 能力 | 预期 |
|------|------|
| Super-LIO mapping | 本地 |
| PCT + SCAN + BT | 本地（不依赖开发机 WiFi） |
| forwarder → Sport | 本地 |
| estop | 本地 |
| Foxglove / SSH 来自办公网 | 中断不影响栈本身 |

---

## D. 尚未实现（仅写状态，勿写成已有）

| 模块 | 状态 |
|------|------|
| VoxelTransaction / Token Adapter | **未实现**（`semantic_token_uplink` 可起，DEV-7 未过） |
| TensorRT BEV 真推理 | **未实现**（`bev_glass_layer` 无 engine） |
| 边端 ActionGroup → PCT 到达 | **未关单**（DEV-6 / EDGE-CL） |
| 面包屑返航真机 | **未关单**（SAFE-1） |
| PCT→BT `/initial_path` 接线 | **板载已工作**（08-19） |
| Elevator-LIO ROS2 跨层 | **等待上游 ROS2** |

---

## E. 可实施性实施例

| 实施例 | 条件 | 结果 |
|--------|------|------|
| lean 栈 H5 2 m | 不经 PCT | PASS（2026-08-07） |
| 边端定向 2 m | ActionGroup NAV，不经 PCT | **10/10** |
| lean 连续 10 min | Step 1 | PASS |
| **板载 PCT 短程 1.0 m** | C++ PCT + `/initial_path` + navi_mode=3 | **PASS 0.43 m（2026-08-19）** |
| 行走中急停 | forwarder + estop | 历史记录 PASS |

---
