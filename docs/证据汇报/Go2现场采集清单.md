# Go2 现场材料与采集清单（N4/N5 补证）

> **历史清单（2026-07-21）**：目标是当时的 N1/N4/N5 与 bag。文中 Nav2 costmap、`/plan`、`NavigateToPose` **不是**当前主链。  
> **当前主链**：`./scripts/motionslam nav start`（PCT C++ + BT `/initial_path` + SCAN）。操作以 [Demo阶段3D_runbook.md](../运维/Demo阶段3D_runbook.md) 为准；验收事实以 [测试记录.md](../测试验收/测试记录.md) 08-19 BOARD-CL 为准。  
> 本趟不做：Token/FST、双机、弱网注入、Rust 网关。

**现场核验快照（2026-07-21 开写本文时）**


| 项       | 【事实】                                                          |
| ------- | ------------------------------------------------------------- |
| HEAD    | `b53c2b6`（working tree **dirty**）                             |
| 分支      | `dev`                                                         |
| eth0    | `${LIDAR_HOST_IP}/24`                                           |
| wlan0   | `${GO2_WLAN_IP}/22`                                           |
| Mid-360 | `${LIDAR_IP}` ping OK                                      |
| 内存      | 15 GiB                                                        |
| 磁盘      | `/` 余量充足（bag 建议预留 ≥20 GB）                                     |
| 镜像      | `motionslam:humble` 存在（`e8a61b4b29e9`）                        |
| 容器名     | 期望 `motionslam`（`run_container.sh` 会创建/启动；到场再 `docker ps -a`） |
| bag 目录  | `~/MotionSLAM_ws/bags/`（已建）                                   |


---

## 0. 本趟成功标准


| 项    | 标准                                     |
| ---- | -------------------------------------- |
| N1   | `check_obstacle_avoid.py` 或 `./scripts/verify_f_series.sh` 再确认 PASS |
| N4   | 2 m 到达，目标误差 **< 0.3 m**                |
| N5   | estop / `stop_nav` 后静止                 |
| 证据   | 完整 bag + `测试记录.md` 追加 + commit/日志快照    |
| Case | ≥5–10 条失败说明 + 1–2 条成功对照（若仍失败，完整失败证据亦可） |


**上次对照（现场对比用）**


| 项      | 上次【事实】             | 本趟  |
| ------ | ------------------ | --- |
| N4 最近距 | 0.46 m（阈值 0.3 m）失败 | ___ |
| 行走距离约  | 1.54 m             | ___ |
| N5     | 部分通过，待复测           | ___ |
| N1     | PASS               | ___ |
| commit | `b53c2b6` dirty    | ___ |


---



## 1. 出发前材料包



### A. 统筹文档（开发机笔记本/平板）


| 材料            | 路径（开发机）                                       | 用途              |
| ------------- | --------------------------------------------- | --------------- |
| 状态看板          | `motion/ProjectState.md`                      | T-004 / Gate 阻塞 |
| Go2 基线        | `motion/40-验证/BENCH-P0-Go2-MotionSLAM-基线.md`  | 对照目标与复现命令       |
| Benchmark 模板  | `motion/40-验证/P0-Benchmark-模板.md`             | 回填实测列           |
| 端侧约束          | `motion/02-架构设计/Go2端侧-可闭环易解释系统工程.md`          | 最小证据包 / 错误码     |
| 数据飞轮          | `motion/02-架构设计/数据飞轮-实施清单与架构.md`              | 采什么、怎么归因        |
| 主机登记          | `motion_ws/hosts.md`                          | SSH / IP        |
| 探测记录          | `motion_ws/docs/go2-host-probe-2026-07-20.md` | 环境已知事实          |
| 上次会话          | `motion_ws/docs/go2-session-2026-07-20.md`    | 上次未跑完项          |
| **本文（Go2 侧）** | `MotionSLAM_ws/docs/Go2现场采集清单.md`             | 现场唯一执行清单        |




### B. 本机工具（开发机）


| 项           | 说明                                                     |
| ----------- | ------------------------------------------------------ |
| SSH         | Host `go2-robot` → `unitree@${GO2_WLAN_IP}`            |
| 快捷脚本        | `motion_ws/scripts/ssh-go2.sh`、`pull-go2-benchmark.sh` |
| BindAddress | 探测约定 `${BIND_ADDRESS}`（本机 IP 变了要改）                      |
| 磁盘          | 预留 bag 空间 ≥ 20–50 GB                                   |




### C. 纸质手记模板（每轮一条）

```text
日期/时间：
commit：
场景：室内同层 / 距离2m / forwarder=true|false
N1：PASS/FAIL
N4：最近距goal=___ m；终态=___；是否进0.3m
N5：estop后是否静止；触发方式=
碰撞：有/无
bag路径：
异常现象（单条）：时间戳/跳变/提前结束/…
对照：成功/失败编号
```

---



## 2. 到场核验（Go2 已有资产）



### A. 硬件 / 网络


| 项       | 【事实】登记值                              | 到场核验           |
| ------- | ------------------------------------ | -------------- |
| 平台      | Unitree Go2 EDU                      | ☐              |
| 边端      | Orin NX，15 GiB                       | ☐ `free -h`    |
| SSH     | `unitree@${GO2_WLAN_IP}`             | ☐ ping + ssh   |
| 机身网     | eth0 `${LIDAR_HOST_IP}/24`（mid360/DDS） | ☐              |
| 办公 WiFi | wlan0 `${GO2_WLAN_IP}/22`            | ☐              |
| Mid-360 | `${LIDAR_IP}`                     | ☐ ping + 话题有数据 |
| App 模式  | AI 模式                                | ☐              |
| 电量 / 场地 | 2 m 直线、无障碍优先                         | ☐              |




### B. 软件栈


| 项      | 路径/值                                                                             | 到场核验                                      |
| ------ | -------------------------------------------------------------------------------- | ----------------------------------------- |
| 工作空间   | `~/MotionSLAM_ws`                                                                | ☐                                         |
| Git    | 分支 `dev`；开测前记录 HEAD + dirty                                                      | ☐ `git status -sb` / `git rev-parse HEAD` |
| 子模块    | Super-LIO / livox / unitree_ros2                                                 | ☐ `git submodule status`                  |
| Docker | 镜像 `motionslam:humble`；容器名以 `run_container.sh` 为准                                | ☐ `docker images` / `docker ps -a`        |
| 流程     | [Demo阶段3D_runbook.md](../运维/Demo阶段3D_runbook.md)                                               | ☐                                         |
| 测试记录   | [测试记录.md](../测试验收/测试记录.md)                                                               | ☐ 本趟追加                                    |
| 脚本     | `check_obstacle_avoid.py`、`verify_scan_planner_h5.py`、`stop_unitree_slam.sh`、`stop_nav.sh` | ☐                                         |
| 急停     | `ros2 run motionslam_pipeline estop_go2`                                         | ☐ 容器内可执行                                  |




### C. 不要指望在 Go2 上找的东西


| 项                  | 说明                       |
| ------------------ | ------------------------ |
| Swarm-SLAM / cslam | Go2 未部署；在开发机 `motion_ws` |
| Zenoh              | 探测时未装                    |
| SLAM-Token / FST   | 未接入；本趟可不采带宽列，但 bag 要有    |


---



## 3. 现场操作（按顺序）



### Step 0：环境快照（开测前必做）

```bash
ssh go2-robot   # 或已在机上
cd ~/MotionSLAM_ws
date -Iseconds
git status -sb
git rev-parse HEAD
git branch --show-current
git submodule status
git diff --stat          # dirty 时必做，勿未记录就 checkout
free -h
docker ps -a
docker images motionslam:humble
```

输出保存为：

```bash
mkdir -p ~/MotionSLAM_ws/docs/sessions
# 例：docs/sessions/session-2026-07-21-precheck.txt
```



### Step 1：释放雷达 + 起容器

```bash
./scripts/stop_unitree_slam.sh
ping -c1 ${LIDAR_IP}
./docker/run_container.sh
```

容器内：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```



### Step 2：按 Demo 流程起栈（先干跑）

```bash
# 干跑（狗不应走）
AUTOSTART=false ./scripts/nav/start_demo_scan_nav.sh with_forwarder:=false
# 或容器内:
# ros2 launch motionslam_bringup demo_scan_stack.launch.py autostart_lifecycle:=false with_forwarder:=false
```

> 历史 Nav2/Hybrid 证据采集见 `feature/hybrid-baseline`（`start_nav.sh` / `start_hybrid_nav.sh` 在 dev 会 REFUSED）。

### Step 3：N1 栈自检

**必须等 launch 起来之后再跑。**

```bash
python3 /ws/scripts/check_obstacle_avoid.py
```

记录：odom Hz、costmap 尺寸、PASS/FAIL。

### Step 4：N4 到达复测（2 m）

停干跑后改真机：

```bash
./scripts/motionslam nav stop
./scripts/motionslam nav start
# N1 再确认后 H5（历史 lean 脚本；PCT 短程请用测试记录 §4 航点）:
python3 /ws/scripts/verify/verify_scan_planner_h5.py --distance 2
```

同时录 bag（§4）。

### Step 5：N5 停栈复测

行走中触发其一，观察是否继续走：

```bash
ros2 run motionslam_pipeline estop_go2
# 或
./scripts/motionslam nav stop
```

记录：触发前速度、触发后是否静止、持续时间（目标 <1 s 停住，见 NAV-3）。

### Step 6：写测试记录（Go2 canonical）

追加到 `~/MotionSLAM_ws/docs/测试验收/测试记录.md`：

- 时间、commit（含 dirty 与否）
- N1 / N4 / N5 结果
- 最近距 goal、终态、bag 路径、异常现象



### Step 7：开发机拉快照

```bash
# 开发机
cd /home/ubuntu/Downloads/motion_ws   # 路径以本机为准
./scripts/pull-go2-benchmark.sh
# bag 需手动 scp（脚本目前只拉 git + 测试记录头）
scp -r go2-robot:~/MotionSLAM_ws/bags/<本次> ./logs/  # 或 bags/go2/
scp go2-robot:~/MotionSLAM_ws/docs/sessions/session-*-precheck.txt ./logs/
```

---



## 4. 必须录的 bag / 日志



### A. 最小话题集（现场 `ros2 topic list` 核对后定稿）

按当前栈常用集合（**2026-08 主链**；07 月 Nav2 话题勿再当必采）：


| 类别         | 建议收录                                           |
| ---------- | ---------------------------------------------- |
| 位姿/里程计     | `/lio/robo/odom`、`/lio/odom`、TF                |
| 点云         | `/lio/cloud_world` |
| 地图/代价      | `/grid_map/occupancy_inflate` |
| 规划         | `/goal_pose`、`/global_path`、`/initial_path`、`/planning/bspline` |
| 控制         | `/motion/command` |
| 编排         | `/demo/mission/event` |
| 安全         | estop 前后终端 log（tee）                            |
| 感知原始（可选）   | `/livox/lidar`、`/livox/imu`（体积大，失败复盘优先）        |


录制建议：

- N4：发 goal 前 5 s → 结束/超时后 5 s
- N5：estop 前后各 ≥ 5 s
- 失败：**保留完整 bag，不要覆盖**

命名：

```text
~/MotionSLAM_ws/bags/YYYYMMDD_N4_2m_<pass|fail>_<commit短哈希>/
~/MotionSLAM_ws/bags/YYYYMMDD_N5_estop_<pass|fail>_<commit短哈希>/
```

示例（ros2 bag，目录名即库名）：

```bash
# 容器内另开终端；话题以现场 list 为准
ros2 bag record -o /ws/bags/$(date +%Y%m%d)_pct_1m_run1_$(git -C /ws rev-parse --short HEAD) \
  /lio/robo/odom /lio/cloud_world /grid_map/occupancy_inflate \
  /goal_pose /global_path /initial_path /planning/bspline /motion/command \
  /demo/mission/event /tf /tf_static
```



### B. 必须带回开发机


| 材料                        | 来源                 | 落到                                   |
| ------------------------- | ------------------ | ------------------------------------ |
| git 状态 + HEAD + submodule | Go2                | `motion_ws/logs/go2-benchmark-*.txt` |
| `docs/测试验收/测试记录.md` 本趟段落       | Go2                | 同日志 + BENCH 更新依据                     |
| N4/N5 bag                 | `bags/`            | `motion_ws/logs/` 或 `bags/go2/`【待建】  |
| precheck.txt              | `docs/sessions/`   | 同日志目录                                |
| 失败 case 手记（5–10 条）        | 纸面/笔记              | 回填 BENCH §4/§5                       |
| 容器/终端日志                   | tee 或 `~/.ros/log` | 一并 scp                               |




### C. 回实验室后更新统筹仓


| 文件                                    | 更新什么                   |
| ------------------------------------- | ---------------------- |
| `40-验证/BENCH-P0-Go2-MotionSLAM-基线.md` | N4/N5 实测、bag 路径、Gate 表 |
| `40-验证/P0-Benchmark-模板.md`            | 实测列、commit、日期          |
| `ProjectState.md`                     | 单机闭环状态、T-004 next      |


---



## 5. 最小证据包勾选（每轮一份）


| 层     | 必采                                   | 本趟有？ |
| ----- | ------------------------------------ | ---- |
| 感知/建图 | 时间戳、odom/TF、（有则）地图版本                 | ☐    |
| 规划    | goal、path、规划器终态（提前结束？）               | ☐    |
| 控制    | cmd_vel / forwarder 输出               | ☐    |
| 安全    | estop 触发与之后运动                        | ☐    |
| 结果    | 最近距 goal、是否进圈、碰撞、任务时间                | ☐    |
| 元数据   | commit、配置（reloc/forwarder）、日期、bag 路径 | ☐    |


无 Token 时通信层/事务版本标【待补充】；**bag 与结果层不能空**。

---



## 6. 现场读数据（不要只看均值）



### A. 单条（失败优先）

- goal 发布时间戳 vs 开始动时间
- 最近距 goal 随时间：提前停 / 偏航 / 冲过头
- BT 事件：`subgoal_dispatched` / `pct_path_received` / `subgoal_reached`（不要再记 Nav2 终态）
- TF / odom 是否跳变
- estop 是否误触发



### B. 对照

- 同场景：`forwarder=false` 干跑 vs `true` 真机
- 若有一次接近成功：保留为对照 bag
- 至少记录：正常期望 vs 本趟特色（例：「1.54 m 处提前结束，最近 0.46 m」）



### C. 分布（样本少也要记）


| 特色项    | 记什么              |
| ------ | ---------------- |
| 到达误差   | 最终 / 最近 / 是否单调接近 |
| 提前结束时刻 | 相对 goal 发布时间     |
| 峰值速度   | cmd_vel 峰值       |
| 停栈延迟   | estop → 静止耗时     |


---



## 7. 安全与现场纪律

1. 宇树固件避障 / Sport 保持开启；测试区留急停人
2. App **AI 模式**
3. 先干跑（`forwarder=false`），再真机（`true`）
4. **dirty working tree**：先记录 `git diff --stat`，勿未记录就 `checkout`
5. 测完：`stop_nav.sh` → 确认不再走 → 再关 launch / 容器（见完整流程 §6）

---



## 8. 一页出门检查表

- [ ] SSH 通（`go2-robot` / `${GO2_WLAN_IP}`）
- [ ] Mid-360 在线 + AI 模式
- [ ] MotionSLAM_ws commit 已记录（含 dirty 与否）
- [ ] `motionslam:humble` 可用；`run_container.sh` 可进
- [ ] 读过 [Demo阶段3D_runbook.md](../运维/Demo阶段3D_runbook.md)
- [ ] N1 PASS
- [ ] N4 2m 已测 + bag 已存
- [ ] N5 estop 已测 + bag 已存
- [ ] [测试记录.md](../测试验收/测试记录.md) 已追加
- [ ] 失败 case 手记 ≥5 条（或成功对照齐全）
- [ ] 开发机 `pull-go2-benchmark.sh` 已跑
- [ ] bag 已 scp 回开发机
- [ ] BENCH / ProjectState 回填计划已排

---



## 9. 双仓路径对照


| 角色                   | 机器       | 路径                    |
| -------------------- | -------- | --------------------- |
| 真机执行 / 测试记录 / bag    | Go2 Orin | `~/MotionSLAM_ws`（本文） |
| 统筹看板 / BENCH / 飞轮    | 开发机      | `motion/`             |
| SSH / pull 脚本 / 探测会话 | 开发机      | `motion_ws/`          |


**总结**：带到现场的完整材料 = 文档口径 + 接入信息 + 复现命令 + bag/日志规范 + 读数据模板 + 回链清单。本趟核心不是开发新功能，而是把 **N4/N5 与可回放证据**补齐，让数据飞轮第一圈转起来。