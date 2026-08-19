# MotionSLAM 运维 Runbook

> 建图、时间校准、外参、重定位细节、Foxglove 排障等。  
> **日常导航（进 Docker → 自检 → 真机 → 停狗）请用：[Demo阶段3D_runbook.md](Demo阶段3D_runbook.md)**  
> **测试填表请用：[测试记录.md](../测试验收/测试记录.md)**

---

## 1. 开机前检查

```bash
date    # 若显示 1970 年, 先修时间 (见下方)
ping -c1 ${LIDAR_IP}          # mid360
ping -c1 ${GO2_ETH_IP}         # 狗主控 (示例)
pgrep -af 'unitree_slam|mid360_driver'  # 应为空 (自研栈前)
```

无线（开发机）:

```bash
sudo ip route add ${LIDAR_SUBNET} via ${GATEWAY_IP}
```

### 系统时间回到 1970

断电/重启后 Jetson RTC 未校准会导致时钟回到 1970，影响 apt、编译、ROS。

```bash
# 容器内 / 宿主机入口
./scripts/motionslam fix-time
# 或: ./scripts/ops/fix_system_time.sh
# 或: date -s "2026-08-19 16:00:00"

# 宿主机 (推荐, 可持久)
sudo timedatectl set-ntp false
sudo timedatectl set-time "2026-07-13 16:00:00"
sudo timedatectl set-ntp true
```

---

## 2. 停用官方 SLAM（必须）

```bash
./scripts/motionslam stop-slam
```

恢复官方栈:

```bash
/unitree/module/unitree_slam/bin/unitree_slam eth0
```

---

## 3. 进入容器并编译

```bash
./docker/run_container.sh
# 容器内
./scripts/motionslam build
source /ws/scripts/dev/source_ws_env.sh
```

**若 `unitree_api` 报 `rosidl_generator_dds_idl` 找不到**：当前容器镜像偏旧，在容器内执行：

```bash
apt-get update && apt-get install -y ros-humble-rosidl-generator-dds-idl
./scripts/dev/build_ws.sh
```

或在宿主机重建镜像后换新容器：

```bash
docker build -t motionslam:humble docker/
# 退出旧容器后重新 ./docker/run_container.sh
```

自动安装（可选）: `MOTIONSLAM_AUTO_APT=1 ./scripts/dev/build_ws.sh`

### 系统时间错误 (apt `not valid yet` / gmake 时间在未来)

容器 **没有 systemd**，容器内执行 `timedatectl` 会报 `PID 1` / `Failed to connect to bus`，属正常现象。时间需改 **宿主机内核时钟**（容器 `--pid host` 与之共享）。

**先确认**（容器内或宿主机均可）:

```bash
date    # 若显示 1970 年或明显不对，必须先修时间
```

**方式 A — 宿主机（推荐）**：`exit` 退出容器后在 Jetson 上:

```bash
sudo timedatectl set-ntp false
sudo timedatectl set-time "2026-07-13 15:00:00"   # 按实际时间改
sudo timedatectl set-ntp true
date
./docker/run_container.sh    # 重新进入，再 apt / build
```

**方式 B — 特权容器内**：`run_container.sh` 已带 `--privileged`，可在容器内:

```bash
date -s "2026-07-13 15:00:00"
date
apt-get update
```

时间正确后，若镜像为新版 (`docker/Dockerfile` 已含 `ros-humble-rosidl-generator-dds-idl`)，可优先 **宿主机重建镜像** 免容器内 apt:

```bash
docker build -t motionslam:humble docker/
```

---

## 4. 建图

**Launch 规范 (容器内 `source /ws/install/setup.bash`):**

| 场景 | Launch |
|------|--------|
| 无头建图 | `ros2 launch motionslam_bringup mapping.launch.py` |
| 建图 + Foxglove + 在线 `/map` 预览 | `ros2 launch motionslam_bringup slam_viz.launch.py` |
| 建图 + 可视化 + **L1 避障**试走 | `ros2 launch motionslam_bringup mapping_walk.launch.py` |
| Demo 自主导航 | `./scripts/motionslam nav start` |

核心组合见 `mapping_stack.launch.py`（`with_forwarder` / `forwarder_backend:=obstacles_avoid`）。  
**勿**单独用 `sport` forwarder 试走（避障/App 状态易不对）。

```bash
# 容器内
ros2 launch motionslam_bringup mapping_stack.launch.py
# 或 建图 + Foxglove
ros2 launch motionslam_bringup slam_viz.launch.py
```

产出: `/ws/maps/map.pcd`

宿主机一键:

```bash
./scripts/motionslam map start
```

### 4.1 慢走建图 + 位姿图后端验收

启动栈后（`mapping.launch.py` 或 `slam_viz.launch.py` 已含 `mapping_motion_guard`、`mapping_keyframe_manager`、`pose_graph_backend`）：

**走法**

- 遥控慢走：线速约 **< 0.35 m/s**、角速 **< 0.45 rad/s**（Violate 时 `/lio/mapping_motion_ok=false`，关键帧会停增）。
- 尽量走 **闭环路线**（同走廊往返），否则只有里程计边、无 `loop closed` 日志。
- 首条回环边需至少 **`min_loop_keyframe_gap`（默认 25）** 个关键帧间隔。

**监控（容器内，已 `source /ws/install/setup.bash`）**

```bash
# 一键：关键帧计数 + motion_ok（每 2s）
bash /ws/scripts/watch_mapping_backend.sh

# 或分开
ros2 topic echo /lio/backend/keyframe_count
ros2 topic echo /lio/backend/loop_closed   # true 表示刚接受一条回环边
ros2 run tf2_ros tf2_echo map world        # 校正 TF；闭环后应缓慢变化，避免阶跃跳变
```

**日志**

- `pose_graph_backend`  stdout：`loop closed i=… j=… sc=… corr=(…m, …deg)` 为接受回环。
- `SC match … but ICP reject`：描述子像但几何不配，可略放宽 SC 或检查是否真回到同地点。
- `loop … rejected: corr too large`：ICP 与 LIO 相对位姿差过大，被 `max_loop_correction_*` 拒边（防误闭环）。

**参数调参**（`motionslam_bringup/config/pipeline.yaml` → `pose_graph_backend`，改后重启 launch）

| 现象 | 建议 |
|------|------|
| 走廊往返仍无回环 | 略**增大** `scan_context_dist_threshold`（如 0.35 → 0.42） |
| 误闭环、地图拉扯、`map→world` 突变 | **减小** `scan_context_dist_threshold`；**加大** `min_loop_keyframe_gap`；或略**收紧** `max_loop_correction_m` / `max_loop_correction_deg` |
| 原地仍刷关键帧/回环 | 确认 `allow_interval_only_keyframes: false`；`min_loop_travel_m` ≥ 2 m |
| 回环太稀 | **减小** `min_loop_keyframe_gap`（注意误匹配风险） |

Foxglove：要看闭环后的全局一致，Fixed Frame 可试 **`map`**（LIO 点云仍在 **`world`**，靠 TF `map→world` 对齐）。

### 4.2 避障失效

**常见根因**：建图时 `cmd_vel_forwarder` 未启或 backend 不是 `obstacles_avoid`；Demo 栈需 `scan_planner_node` + forwarder 在线。

```bash
docker exec motionslam bash -lc 'source /ws/install/setup.bash && python3 /ws/scripts/check_obstacle_avoid.py'
```

| 现象 | 处理 |
|------|------|
| 无 `cmd_vel_forwarder` | `./scripts/motionslam nav stop` 后再 `nav start`，或 `mapping_walk.launch.py` |
| backend=sport | Demo 默认 sport；建图试走用 `forwarder_backend:=obstacles_avoid` |
| 多个 livox / forwarder 同名 | `stop_nav.sh` 杀净再启 |
| 急停后不避 | 升级后 `estop_go2` 双通道零速；再发 `/cmd_vel` 会重 enable |

---

## 5. 导航（PCT C++ + SCAN 主链）

**完整步骤见 [Demo阶段3D_runbook.md](Demo阶段3D_runbook.md)。** 结果填表 → **[测试记录.md](../测试验收/测试记录.md)**。

```bash
./scripts/motionslam nav start
./scripts/motionslam nav watch
# 容器内观测（可选）
python3 /ws/scripts/verify/verify_scan_planner_h5.py --distance 2
./scripts/motionslam nav stop
ros2 run motionslam_pipeline estop_go2
```

`nav start` 已默认 `WITH_PCT_PLANNER=true`。不存在 `nav start-lean`。干跑加 `with_forwarder:=false`。

---

## 6. 重定位（离线 SC-PGO）

> dev 主线为 Super-LIO **mapping 模式**，不依赖 Global ICP / `relocation.launch.py`。本节供离线 SC-PGO 与历史地图工具参考。

**大地图 OOM:** `map.pcd` 过大时 `relocation_node` 会被内核杀死。先降采样:

```bash
bash scripts/ensure_nav_maps.sh
# 或: python3 scripts/downsample_reloc_map.py --leaf 0.5
# 产出 maps/map_reloc.pcd (super_lio_reloc_mid360.yaml 默认读取)
```

离线 SC-PGO / 历史 reloc 启动（可选）:

```bash
python3 scripts/offline/run_sc_pgo_offline.py --session ./sessions/sc_pgo_xxx
```

改 yaml 初值（ICP 已跑完时）— 编辑 `config/super_lio_reloc_mid360.yaml`:

```yaml
lio.relocation.init_pose: [x, y, z, roll_deg, pitch_deg, yaw_deg]
```

认位脚本: `python3 scripts/read_reloc_init_pose.py` · 工位保存: `python3 scripts/save_reloc_spot.py`

---

## 7. Foxglove

- 连接: `ws://${GO2_IP}:8765`（把 IP 换成当前 NX 在开发机可达的地址）
- **Fixed Frame = `world`**
- 局部占据: `/grid_map/occupancy_inflate`
- 规划轨迹: `/planning/trajectory_path`、`/planning/bspline`、`/initial_path`
- PCT 全局: `/global_path`（C++ 常驻不发 `/tomogram`）

日常 Demo 栈经 `demo_scan_stack` 起 Foxglove 时 **`with_legacy_bridge:=false`，只有 8765**。单独 `viz.launch.py` 才可能再开 8766。

### 7.1 `handshake failed` 排查

单独起 `viz.launch.py` 时可提供两个端口（默认 `with_legacy_bridge:=false`）：

| 端口 | 协议 | 适用客户端 |
|------|------|------------|
| **8765** | `foxglove.sdk.v1` | 最新 Foxglove Studio、https://app.foxglove.dev |
| **8766** | `foxglove.websocket.v1` | 旧版 Foxglove、Lichtblick 等 |

若日志里 8765 反复 `handshake failed`，**开发机请改连**：

```text
ws://<NX_IP>:8766
```

容器内自检：

```bash
bash /ws/scripts/check_foxglove_ws.sh 127.0.0.1
# 或从开发机
bash scripts/check_foxglove_ws.sh ${LIDAR_HOST_IP}
```

开启 bridge 调试：

```bash
ros2 launch motionslam_bringup viz.launch.py debug:=true
```

仅保留新版 bridge（不需要 8766）：

```bash
ros2 launch motionslam_bringup viz.launch.py with_legacy_bridge:=false
```

### 7.2 网络

开发机 IP 示例 `192.168.108.x` 需能路由到狗/NX；确认 `ping <NX_IP>` 与 `curl -v --no-buffer -H 'Connection: Upgrade' -H 'Upgrade: websocket' -H 'Sec-WebSocket-Version: 13' -H 'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==' -H 'Sec-WebSocket-Protocol: foxglove.sdk.v1' http://<NX_IP>:8765/` 能返回 101 Switching Protocols。

- 详见 [Foxglove代价图可视化](../开发参考/Foxglove代价图可视化.md)

---

## 8. 外参标定 (T2.5)

完整参考文件、话题、迭代记录见 **[开发参考/T2.5_odom_robo外参标定.md](../开发参考/T2.5_odom_robo外参标定.md)**。

```bash
# 容器内 /ws，Super-LIO 栈运行中
python3 scripts/calibrate_odom_robo.py --once
```

---

## 9. E2E / F 系列冒烟

F 系列是 **LIO/点云/bspline 前段** 工具，**不是**当前 PCT 主链验收。`ego_nav*.launch.py` 已从 dev 移除；当前栈用 `demo_scan_stack.launch.py`。Nav2/ego 完整 F 表见 `feature/hybrid-baseline`。

### 宿主机 vs 容器

| 你在哪 | 做什么 |
|--------|--------|
| `unitree@ubuntu:~/MotionSLAM_ws` | `./docker/run_container.sh` 进容器 |
| `root@ubuntu:/ws`（已在容器内） | **不要**再跑 `run_container.sh`（容器里没有 docker） |

```bash
./scripts/motionslam verify smoke
./scripts/motionslam verify f-series --full
# 主链到达（SCAN H5，需已开栈）
./scripts/motionslam verify h5 --distance 2
```

主链板载到达以 [测试记录.md](../测试验收/测试记录.md) BOARD-CL 为准，不要把 F 系列或历史 N4/N5 当成当前 PCT 闭环证据。详见 [F系列前段验收.md](../开发参考/F系列前段验收.md)。

---

## 10. 急停 / 安全

- 干跑: `with_forwarder:=false`
- 看门狗: `cmd_vel_forwarder` 400ms, `pos_cmd_to_twist` 500ms 超时零速
- 首轮真机 `vx_max=0.5` (`pipeline.yaml`)
- 急停: 发布 `/waypoint_fsm/stop` 或 Ctrl+C launch；确认 `/cmd_vel` 归零
