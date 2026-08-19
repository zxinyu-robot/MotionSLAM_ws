# 私人 GitHub 备份清单

> **用途**：个人私有仓归档本项目中「能讲清楚、体积小、有增量」的部分。  
> **不是** 把 `MotionSLAM_ws` 整仓镜像上去。  
> **权属**：若代码属于实验室/公司，私有仓仍可能侵权。未获书面许可前不要 `git push`。本文只规定**切什么、不切什么**。

对照：[readme.md](../../readme.md) 目录 · 验收事实只认 [测试记录.md](../测试验收/测试记录.md)。

---

## 0. 原则

1. **整仓是集成工作区**，不是一份可展示的产品：上游 LIO/驱动/SCAN/PCT 已有公开 GitHub。
2. 私人备份做成 **两个小仓**：能跑的切片（仓 A）+ 笔记（仓 B）。不要把目标态 OS 写进仓 A 的 README 当「已交付」。
3. 仓 A **不是**可一键上狗的完整工程。后续开发必须按 **§9 框架/开源 + §10 耦合迁移** 重建工作区，用话题契约把切片接回上游。
4. 上传前去掉现场网段、bag、engine、机身密钥。
5. 面试只打开仓 A 里 **三个文件**（见 §6），不要用未关单能力当作品。

---

## 1. 仓怎么切

### 仓 A：`motionslam-nav-core`（主备份，private）

自研导航内核 + 编排 + 边端契约 + 起停骨架。子模块 **只写 URL**，不 vendoring Super-LIO / SCAN / unitree_ros2。

建议目录：

```text
README.md
THIRD_PARTY.md   # 上游 URL + commit SHA（迁移必用）
LICENSE          # 仅在你有权声明时添加；否则写 UNLICENSED / 内部备份
docker/          ← Dockerfile + cyclonedds.xml（去现场 bind IP）
pipeline/        ← 自 src/motionslam_pipeline 精选
msgs/            ← src/motionslam_msgs 整包
ipc/             ← src/motionslam_ipc 整包
bringup/         ← scripts + 关键 test + 少量 config（去 IP）
docs/            ← 接口规范 + Qwen 接入（去 IP）+ 本清单
scripts/         ← nav start/stop 骨架（去现场 IP）
patches/         ← SCAN / PCT 的 diff，不是整棵上游树
```

### 仓 B：`motionslam-notes`（可选，private）

架构目标态、规划卡点、测试数字摘录。给自己复盘，避免和「能跑的代码」混在一个 README 里。

```text
README.md                    # 写明：目标态 ≠ 已交付
architecture-os.md           # 三空间 + 与 RoboOS 对照（从架构文档摘）
planning-demo1.md            # 卡点顺序摘录
acceptance-0819.md           # BOARD-CL 数字，不附 bag
```

---

## 2. 仓 A：建议拷贝的文件

路径相对本仓库根。用 `rsync` 或手动拷；**不要** `git clone --mirror` 整仓。

### 2.1 pipeline（高优先级）

| 源路径 | 说明 |
|--------|------|
| `src/motionslam_pipeline/CMakeLists.txt` | 目标列表；完整编过还需 `motionslam_msgs`、`motionslam_ipc` |
| `src/motionslam_pipeline/package.xml` | |
| `src/motionslam_msgs/`（整包） | `MotionCommand.msg`；forwarder / closed_loop 共用 |
| `src/motionslam_ipc/`（整包） | POSIX SHM 点云环；pipeline 依赖 |
| `src/motionslam_pipeline/src/PctResidentPlanner.cpp` | C++ 常驻 A*，不写执行速度 |
| `src/motionslam_pipeline/src/CmdVelForwarder.cpp` | Sport 转发、限幅 |
| `src/motionslam_pipeline/include/motionslam/VelocityLimiter.hpp` | |
| `src/motionslam_pipeline/include/motionslam/WatchdogPolicy.hpp` | |
| `src/motionslam_pipeline/src/VirtualObstacleMerge.cpp` | 玻璃等虚拟障碍并入点云 |
| `src/motionslam_pipeline/src/EstopGo2.cpp` | 急停 |
| `src/motionslam_pipeline/test/test_cmd_vel_forwarder.cpp` | |
| `src/motionslam_pipeline/test/test_pos_cmd_timeout.cpp` | |

**可选（与导航主链弱相关，按需）：** `BevGlassLayer.cpp`（无 engine 不算能力）、`DemoNavigationContextNode.cpp`、`MapSessionManager.cpp`、`PoseGraphBackend.cpp`、`LoopDetectionNode.cpp`、`LocalMapNode.cpp`。Go2 使能/恢复遥控（`EnableGo2ApiWalk.cpp`、`RestoreGo2Remote.cpp`）含机身细节，**默认不拷**，除非已确认可带走。

### 2.2 bringup 脚本与单测（高优先级）

| 源路径 | 说明 |
|--------|------|
| `src/motionslam_bringup/scripts/demo_scan_bt_orchestrator.py` | 唯一 `/initial_path` 写口 |
| `src/motionslam_bringup/scripts/return_home.py` | 面包屑倒序 |
| `src/motionslam_bringup/scripts/behavior_mode.py` | 物模型信封校验 |
| `src/motionslam_bringup/scripts/task_context.py` | session/代数/lifecycle |
| `src/motionslam_bringup/scripts/action_group.py` | 旧 schema 兼容 |
| `src/motionslam_bringup/scripts/directive_receiver_node.py` | `:9879` |
| `src/motionslam_bringup/scripts/command_executor_node.py` | 抽出 Navigate xyz |
| `src/motionslam_bringup/scripts/execution_feedback.py` | |
| `src/motionslam_bringup/scripts/execution_feedback_node.py` | `:9880` |
| `src/motionslam_bringup/scripts/data_layer_types.py` | 层定义 |
| `src/motionslam_bringup/scripts/rgb_forward_frame.py` | MSRGB |
| `src/motionslam_bringup/scripts/subgraph_snapshot.py` | 子图 JSON |
| `src/motionslam_bringup/test/test_behavior_mode.py` | |
| `src/motionslam_bringup/test/test_action_group.py` | |
| `src/motionslam_bringup/test/test_return_home.py` | |
| `src/motionslam_bringup/test/test_plan_fail_recovery.py` | |

**可选：** `rgb_keyframe_uplink_node.py`、`subgraph_publisher_node.py`、`semantic_token_uplink_node.py`、`semantic_bev_frame.py`、`edge_uplink_client.py`（边端物料面）。launch 只拷 `demo_scan_stack.launch.py` 作参考，依赖大量外部包，私人仓 **不必能 colcon 编过**。

### 2.3 配置（必须脱敏后再拷）

| 源路径 | 处理 |
|--------|------|
| `src/motionslam_bringup/config/edge_uplink.yaml` | `edge_host` 改为 `127.0.0.1` 或 `${EDGE_HOST}` |
| `src/motionslam_bringup/config/acceptance_short_waypoints.yaml` | 可拷（短程验收航点） |
| `src/motionslam_bringup/config/MID360_config.json` | **不要拷**（现场雷达 IP） |

### 2.4 文档（去 IP）

| 源路径 | 处理 |
|--------|------|
| `docs/项目规范/边端协同接口规范.md` | 替换 `192.168.*` |
| `docs/开发参考/边侧Qwen2.5-VL接入说明.md` | 同上 |
| `docs/开发参考/PCT-Planner接入说明.md` | 可拷 |
| 本文 | 可放入仓 A `docs/` |

**不要整份拷：** `docs/证据汇报/`、`docs/sessions/`、含现场拓扑的 runbook 全文。仓 B 只摘测试记录表格。

### 2.5 脚本骨架

| 源路径 | 处理 |
|--------|------|
| `scripts/offline/thing_envelope_qwen_navigate.json` | 可拷 |
| `scripts/offline/send_thing_envelope.py` | host 默认 `127.0.0.1` |
| `scripts/nav/start_demo_scan_nav.sh` | 作参考；依赖本仓 Docker/路径 |
| `scripts/motionslam` | 可选，体量大则只留 README 里三行命令 |

### 2.6 上游 diff（不要整树）

在本仓对子模块做 `git diff`，输出放到仓 A `patches/`：

```bash
# 示例：SCAN 相对其 submodule 记录的改动
cd src/SCAN-Planner && git diff > ../../patches/scan-planner-motionslam.patch
cd src/PCT-Planner && git diff > ../../patches/pct-planner-motionslam.patch
```

README 写清上游：

- Super-LIO：`https://github.com/Liansheng-Wang/Super-LIO.git`（`ros2`）
- SCAN-Planner：`https://github.com/wuyi2121/SCAN-Planner.git`（`ros2-community`）
- PCT_planner：`https://github.com/VectorRobotics/PCT_planner.git`
- unitree_ros2 / livox_ros_driver2：官方仓

私人仓 **不** 包含 `src/Super-LIO`、`src/livox_ros_driver2`、`src/unitree_ros2`、`src/PCT-Planner` 全量、`src/SCAN-Planner` 全量、`scan_planner_ws/`、`pct_planner_ws/`。

---

## 3. 明确不要进 GitHub（即使 private）

| 类别 | 例子 |
|------|------|
| 现场网段 | `192.168.110.*`、`192.168.123.*`、`EDGE_HOST`、狗 SN、`go2_001` 若为真实序列号则改占位 |
| 雷达/机身配置 | `MID360_config.json` 真 IP |
| 大数据 | `*.bag` `*.mcap` `*.pcd` `logs/` `docs/sessions/` `maps/` |
| 模型 | TensorRT engine、权重、未授权 BEV |
| 构建产物 | `build/` `install/` `log/` |
| 编辑器 | `.cursor/` `.vscode/` 含绝对路径的 workspace |
| 密钥 | 任何 token、password、device_secret |

上传前在**待推目录**执行：

```bash
grep -RInE '192\.168\.|password|secret|api_key' --exclude-dir=.git .
```

有命中则改成 `EDGE_HOST` / `GO2_IP` / `LIDAR_IP` 占位符。

---

## 4. 建议拷贝命令（示例）

在获得许可后，于本机另开目录（**不要**在狗端 `MotionSLAM_ws` 里改 remote 指向你的私人仓）。

```bash
SRC=~/MotionSLAM_ws
DST=~/motionslam-nav-core   # 空目录，将 git init

mkdir -p "$DST"/{pipeline/{src,include/motionslam,test},bringup/{scripts,test,config},msgs,ipc,docs,scripts/offline,patches,docker}

cp -a "$SRC"/src/motionslam_msgs/. "$DST"/msgs/
cp -a "$SRC"/src/motionslam_ipc/. "$DST"/ipc/
cp "$SRC"/docker/Dockerfile "$SRC"/docker/cyclonedds.xml "$DST"/docker/
# 可选：run_container.sh 去掉硬编码现场 IP 后再拷

# pipeline
cp "$SRC"/src/motionslam_pipeline/CMakeLists.txt "$SRC"/src/motionslam_pipeline/package.xml "$DST"/pipeline/
cp "$SRC"/src/motionslam_pipeline/src/PctResidentPlanner.cpp "$DST"/pipeline/src/
cp "$SRC"/src/motionslam_pipeline/src/CmdVelForwarder.cpp "$DST"/pipeline/src/
cp "$SRC"/src/motionslam_pipeline/src/VirtualObstacleMerge.cpp "$DST"/pipeline/src/
cp "$SRC"/src/motionslam_pipeline/src/EstopGo2.cpp "$DST"/pipeline/src/
cp "$SRC"/src/motionslam_pipeline/include/motionslam/VelocityLimiter.hpp "$DST"/pipeline/include/motionslam/
cp "$SRC"/src/motionslam_pipeline/include/motionslam/WatchdogPolicy.hpp "$DST"/pipeline/include/motionslam/
cp "$SRC"/src/motionslam_pipeline/test/test_cmd_vel_forwarder.cpp "$DST"/pipeline/test/
cp "$SRC"/src/motionslam_pipeline/test/test_pos_cmd_timeout.cpp "$DST"/pipeline/test/

# bringup
cp "$SRC"/src/motionslam_bringup/scripts/{demo_scan_bt_orchestrator,return_home,behavior_mode,task_context,action_group}.py "$DST"/bringup/scripts/
cp "$SRC"/src/motionslam_bringup/scripts/{directive_receiver_node,command_executor_node,execution_feedback,execution_feedback_node,data_layer_types,rgb_forward_frame,subgraph_snapshot}.py "$DST"/bringup/scripts/
cp "$SRC"/src/motionslam_bringup/test/test_{behavior_mode,action_group,return_home,plan_fail_recovery}.py "$DST"/bringup/test/

# 样例 JSON（send 脚本默认 host 已是 127.0.0.1）
cp "$SRC"/scripts/offline/thing_envelope_qwen_navigate.json "$SRC"/scripts/offline/send_thing_envelope.py "$DST"/scripts/offline/

# 文档：拷完后必须跑 §3 的 grep
cp "$SRC"/docs/项目规范/边端协同接口规范.md "$DST"/docs/
cp "$SRC"/docs/开发参考/边侧Qwen2.5-VL接入说明.md "$DST"/docs/
cp "$SRC"/docs/开发参考/私人GitHub备份清单.md "$DST"/docs/
```

拷完后：改 yaml/md 中的 IP → `git init` → 确认 `git status` 无 bag/log → 再 `git remote add` 私人仓。

仓 A **不必** 在私人 GitHub 上能一键编过整机；能过 `python3 -m pytest bringup/test/test_behavior_mode.py` 即可作为契约切片的最低自检。要继续沿现网链路开发，按 **§10** 重建完整工作区。

---

## 5. 仓 A README 应写什么（模板）

```markdown
# motionslam-nav-core（私人备份）

切片自 Go2 + Orin NX 导航栈，**不是**完整可部署工作区。

## 主链（已验收）

Super-LIO → pct_resident_planner → BT /initial_path → SCAN navi_mode=3 → Sport

真机：2026-08-19 板载短程 1.0 m，偏差 0.43 m。

## 明确未关单

边端 JSON 到达、返航/玻璃真机、Token、PolicyDB、Qwen 在环。

## 上游与重建

开源依赖、框架版本、如何把本仓耦合成完整工作区：见备份清单 **§9–§10**。

## 约束

世界空间 JSON 不得含 cmd_vel；规划器不得直写执行速度。
```

不要写「智源级 OS 已落地」「厘米级导航」「多机协同」。

---

## 6. 面试只打开这三个文件

| 顺序 | 文件 | 一句话 |
|------|------|--------|
| 1 | `PctResidentPlanner.cpp` | 引导内核，只出路径，不写速度 |
| 2 | `demo_scan_bt_orchestrator.py` | stamp 校验后写 `/initial_path`，拒空/伪短路径 |
| 3 | `behavior_mode.py` | 拒 `cmd_vel`；要 robot × scene × mode |

返航/玻璃/EDGE-CL/Qwen：可以说「代码或契约在，真机未关单」。

---

## 7. 推送前核对

- [ ] 已确认可以个人备份该切片
- [ ] 无 `192.168.`、无 bag/pcd/engine
- [ ] README 区分已验收 / 未关单
- [ ] 无完整 SCAN / Super-LIO / unitree_ros2 树
- [ ] 已写 `THIRD_PARTY.md`（`git submodule status`）
- [ ] 已导出 SCAN `git diff` → `patches/`
- [ ] remote 是 **private**，未勾选 public
- [ ] 未把狗端 `origin` 改成私人仓（现场仓与备份仓分离）

---

## 8. 不值得单独建仓的

整份 `SCAN-Planner`、`PCT-Planner`、`Super-LIO`、Foxglove/DDS 长文、证据包 PPT 口径、未脱敏 runbook。需要时在仓 B 用三行摘录 + 指回本机路径即可。

---

## 9. 后续开发要用的框架与开源

切片仓只保存 **自研增量**。继续沿现网主链开发，必须在完整工作区里编、跑、验；框架与开源如下。日常 **不用 Nav2 作规划链**。

### 9.1 运行与构建框架（现网）

| 层 | 现网选择 | 备注 |
|----|----------|------|
| 主机 | Jetson Orin NX，Ubuntu 20.04 / JetPack 5 | 宿主机不满足 Super-LIO ros2 的 C++20，**栈跑在容器里** |
| 容器 | `ros:humble` → 镜像 `motionslam:humble` | [docker/Dockerfile](../../docker/Dockerfile) |
| 中间件 | ROS 2 Humble + **RMW CycloneDDS** | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，`ROS_DOMAIN_ID=0` |
| DDS 配置 | `docker/cyclonedds.xml` | 与 Go2 机身 DDS **域 0 原生互通** |
| 构建 | colcon；**双工作区** | 主仓 `/ws` + SCAN overlay `/ws/scan_planner_ws` |
| 环境 | `scripts/dev/source_ws_env.sh` | 先 Humble → 主 `install` → `scan_planner_ws/install` |
| 点云 | PCL + `pcl_conversions` / `pcl_ros` | Super-LIO / PCT / merge |
| 线性代数 | Eigen3、glog、TBB、yaml-cpp | Dockerfile apt |
| 雷达 SDK | [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | 镜像内编译安装，再编 `livox_ros_driver2` |
| 可视化 | `ros-humble-foxglove-bridge`（8765） | 可选 legacy 0.8.2 → 8766；**不是**导航依赖 |
| 消息生成 | `rosidl` + `ros-humble-rosidl-generator-dds-idl` | 编 `unitree_api` 需要 |
| 语言 | C++（pipeline、SCAN、PCT 常驻）、Python 3（BT、边端 JSON） | 日常开栈不依赖 Python/Open3D PCT |
| 边侧 VL（可选） | Qwen2.5-VL-7B **在边侧 PC** | 不进 Orin 实时决策；契约见 Qwen 接入说明 |

**不要**为了续作主链去装 Nav2 全栈。镜像里 `nav2-msgs` 仅历史对照脚本。

### 9.2 开源工作（子模块 / 上游）

钉死 URL + **迁移时记录 commit SHA**（写入仓 A `THIRD_PARTY.md`）。

| 开源 | URL / 分支 | 在主链中的角色 | 与仓 A 的关系 |
|------|------------|----------------|---------------|
| Super-LIO | `Liansheng-Wang/Super-LIO` `ros2` | 无先验 mapping：`/lio/robo/odom`、`/lio/cloud_world` | 仓 A 不入库；工作区 submodule |
| livox_ros_driver2 | `Livox-SDK/livox_ros_driver2` | Mid-360 驱动 | 同上；编前拷 `package_ROS2.xml` → `package.xml` |
| unitree_ros2 | `unitreerobotics/unitree_ros2` | `unitree_go` / `unitree_api`、Sport | forwarder 依赖；仓 A 不入库 |
| SCAN-Planner | `wuyi2121/SCAN-Planner` `ros2-community` | 局部 B-spline + `closed_loop_controller` | **独立 colcon 工作区**；仓 A 只放 `patches/` |
| PCT_planner | `VectorRobotics/PCT_planner` `main` | Python/Open3D 对照 | **日常不启**；C++ 常驻在仓 A 的 `PctResidentPlanner.cpp` |
| Livox-SDK2 | 见上 | 驱动前置 | 进 Docker 镜像，不进仓 A |
| foxglove/ros-foxglove-bridge | Humble apt + 可选 tag `0.8.2` | 可视化 | 不进仓 A |

自研包（仓 A 应带走、工作区要能 `colcon`）：`motionslam_msgs`、`motionslam_ipc`、`motionslam_pipeline`、`motionslam_bringup`。

### 9.3 主链耦合面（话题 / 服务，不要改名）

续作时 **先保持这些名字**，再换实现。这是切片与上游的 ABI。

```text
Mid-360 → livox → Super-LIO
                    /lio/cloud_world  /lio/robo/odom  /lio/backend/context_generation
                    → pct_resident_planner
                         /goal_pose  ← BT
                         /global_path → BT（stamp 必须等于请求）
                    → demo_scan_bt_orchestrator
                         /initial_path → SCAN navi_mode=3
                    → scan_planner_node
                         /planning/bspline → closed_loop_controller
                         /motion/command → cmd_vel_forwarder → Sport（CycloneDDS 域 0）

边端（可选）：Edge TCP 9877/9878/9880 ← 狗连边；9879 → directive_receiver
              JSON → /semantic/action_group → command_executor → /demo/mission/semantic_goal → BT
```

硬约束（换开源实现也不能破）：

- PCT **只**出 `/global_path`，禁止写 `/initial_path` 或 `/cmd_vel`
- BT 是 **唯一** 执行路径写口
- 边端 JSON **禁止** `cmd_vel` / `twist`
- SCAN 日常 `fsm.navi_mode=3` 跟 `/initial_path`，不是 `move_base_simple/goal`

### 9.4 按能力继续开发时还要什么

| 你要做的 | 额外开源 / 框架 | 仍走哪条耦合 |
|----------|-----------------|--------------|
| 板载 Navigate 增强（重规划、动态障碍） | 现有 Super-LIO + SCAN + 仓 A PCT/BT | `/global_path` stamp、路径头可通行 |
| 返航 / 玻璃真机关单 | 仓 A `return_home` + `virtual_obstacle_merge` | 同一 `/initial_path` |
| 边端 Navigate（EDGE-CL） | 仓 A 物模型 + 边侧监听 9876–9880 | `thing_envelope.v1` → `:9879` |
| Qwen2.5-VL 决策 | 边侧推理栈（Transformers / vLLM 等，**自选**） | 只装配信封；xyz 来自 9877 frontier |
| Token / BEV | TensorRT + 仓内 `bev_glass_layer`（无 engine 不算） | `:9876` MSTOK；不进控制 JSON |
| 换局部规划器 | 任意能订 `/initial_path`、出跟踪轨迹的节点 | **不要**让新规划器直写 Sport |

Python PCT 工作区仅对照调试（`with_pct_python`），与 C++ `/build_tomogram` 冲突，续作主链 **不要**当默认。

---

## 10. 切片仓如何耦合成完整工程（迁移）

目标：在新机器上得到与现网同构的工作区，而不是「只克隆仓 A 就开狗」。

```text
公开上游（git submodule，钉 SHA）
        + 仓 A patches/ 打到 SCAN（及可选 PCT Python）
        + 仓 A 覆盖/放入 src/motionslam_*
        + docker/（Dockerfile + cyclonedds.xml）
        → colcon 主仓 + colcon scan_planner_ws
        → source_ws_env.sh
        → 与现网相同话题契约
```

### 10.1 仓 A 钉版本：`THIRD_PARTY.md`

迁移时 SHA 比分支名更重要。在本仓生成：

```bash
git submodule status
```

把输出写入仓 A `THIRD_PARTY.md`，表格示例：

| 组件 | 远程 | 分支 | commit |
|------|------|------|--------|
| Super-LIO | github.com/Liansheng-Wang/Super-LIO | ros2 | （填 SHA） |
| livox_ros_driver2 | github.com/Livox-SDK/livox_ros_driver2 | （默认） | |
| unitree_ros2 | github.com/unitreerobotics/unitree_ros2 | （默认） | |
| SCAN-Planner | github.com/wuyi2121/SCAN-Planner | ros2-community | |
| PCT_planner | github.com/VectorRobotics/PCT_planner | main | 可选 |
| Livox-SDK2 | Dockerfile 内 clone | — | depth 1，以镜像层为准 |

### 10.2 重建工作区步骤

在新机（建议仍用 Docker）：

```bash
# 1) 空工作区骨架
mkdir -p MotionSLAM_ws/src MotionSLAM_ws/scan_planner_ws/src
cd MotionSLAM_ws
# 放入仓 A 的 docker/（Dockerfile + cyclonedds.xml）

# 2) 上游 submodule（SHA 来自 THIRD_PARTY.md）
git submodule add -b ros2 https://github.com/Liansheng-Wang/Super-LIO.git src/Super-LIO
git submodule add https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
git submodule add https://github.com/unitreerobotics/unitree_ros2.git src/unitree_ros2
git submodule add -b ros2-community https://github.com/wuyi2121/SCAN-Planner.git src/SCAN-Planner
git submodule update --init --recursive
# 逐个 checkout 到钉死的 SHA

# 3) 自研包：仓 A → 现网包布局（见 §11，不要改 ROS 包名）

# 4) SCAN overlay + patch
#    现网：SCAN 在 scan_planner_ws 编译；主仓 colcon 跳过同名包（scripts/dev/build_ws.sh）
ln -sfn ../../src/SCAN-Planner scan_planner_ws/src/SCAN-Planner
cd src/SCAN-Planner && git apply ../../patches/scan-planner-motionslam.patch && cd ../..

# 5) 镜像与编译（容器内，工作区挂载为 /ws）
docker build -t motionslam:humble docker/
./scripts/dev/build_ws.sh
cd scan_planner_ws && colcon build --symlink-install && cd ..
source scripts/dev/source_ws_env.sh
```

Livox 包在 colcon 前必须：`cp package_ROS2.xml package.xml`（`build_ws.sh` 已做）。缺 `ros-humble-rosidl-generator-dds-idl` 则 `unitree_api` 编不过。

### 10.3 包名与 overlay（完整性要点）

| 工作区 | 编什么 | 不编什么 |
|--------|--------|----------|
| `/ws`（主） | Super-LIO、livox、unitree_ros2、`motionslam_*` | SCAN 包名（避免与 overlay 双份） |
| `/ws/scan_planner_ws` | `scan_planner`、`plan_env`、`traj_utils` 等 | motionslam / LIO |
| `/ws/pct_planner_ws` | 仅调试 Python PCT | 日常 `nav start` 不 source 也行 |

`motionslam_bringup` 的 `exec_depend` 含 `scan_planner`、`super_lio`、`unitree_go`：launch 能起，取决于 **两个 install 都 source**。只克隆仓 A、不编 SCAN overlay，栈不完整。

### 10.4 开发时改哪一侧

| 改动 | 提交到 | 如何保持可迁移 |
|------|--------|----------------|
| PCT 常驻、forwarder、merge、BT、物模型 JSON | **仓 A** | 包名、话题名不变 |
| SCAN 局部 / navi_mode=3 / `/initial_path` 回调 | 上游 SCAN 的 **patch 增量** | 对钉死 SHA 重新 `git diff` 覆盖 `patches/scan-planner-motionslam.patch` |
| Super-LIO / 驱动 / Sport 消息 | 尽量不改；若必须 | 新 patch 文件 + 更新 `THIRD_PARTY.md` SHA |
| Docker 依赖、DDS | 仓 A `docker/` | 新机先重建镜像再 colcon |
| 边侧 Qwen | **边侧自己的仓** | 只依赖 `:9879` 信封与 `:9877/:9878/:9880` 物料；不链进 Orin 包 |

双向同步：现场 `MotionSLAM_ws` 继续当集成仓；私人仓 A 定期 `rsync` 自研目录。**禁止**把私人 `origin` 设成狗端唯一 remote。

### 10.5 迁移后最小自检

1. `python3 -m pytest` 仓 A 的 `test_behavior_mode.py`（无 ROS 也可）
2. 容器内 `ros2 pkg prefix scan_planner` 与 `motionslam_pipeline` 都有 install
3. 硬件在线时 `ros2 topic list` 能看到机身或 Super-LIO 话题
4. 干跑：`with_forwarder:=false` 时 PCT 出 `/global_path`、BT 不写直线抢跑
5. 有狗再验 BOARD-CL 同类短程；**不要**假设私人仓克隆后等于 08-19 真机已过

缺 SHA、缺 overlay、缺 CycloneDDS 域 0，都会表现为「代码在、狗不动或 stamp 对不上」。

---

## 11. 仓 A 目录与完整工作区对照

拷贝时按右列落盘，colcon 才认包。

| 仓 A | 完整工作区 |
|------|------------|
| `pipeline/` | `src/motionslam_pipeline/` |
| `msgs/` | `src/motionslam_msgs/` |
| `ipc/` | `src/motionslam_ipc/` |
| `bringup/` | `src/motionslam_bringup/`（scripts / test / config） |
| `patches/scan-planner-motionslam.patch` | 应用到 `src/SCAN-Planner` 后，在 `scan_planner_ws` 编译 |
| `docker/` | 工作区根 `docker/` |
| `THIRD_PARTY.md` | 生成 `.gitmodules` + checkout SHA |
| （无） | `src/Super-LIO`、`livox_ros_driver2`、`unitree_ros2` 用 submodule 拉 |

§4 的 `cp` 已含 msgs / ipc / docker。漏掉 msgs 或 ipc，pipeline **无法在新工作区单独编译**。
