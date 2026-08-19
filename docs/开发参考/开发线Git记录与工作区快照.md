# 开发线 Git 记录与工作区快照

> **快照时间**：2026-08-19（GitHub 同步：`8742ba0` 之后工作区与备份仓文件一致；`_inbox` 草稿已纳入备份）
> **狗端工作区**：`~/MotionSLAM_ws`（分支 `dev`）  
> **GitHub 私人备份**：`~/MotionSLAM_ws_github`（分支 `main`）  
> **用途**：记录 GitLab 集成线 commit 演进、GitHub 备份线状态、以及**尚未进入任何 remote 的工作区改动**。验收事实仍以 [测试记录.md](../测试验收/测试记录.md) 为准。

---

## 1. 两条 Git 线（不要混为一谈）

```
┌─────────────────────────────────────────────────────────────┐
│  ~/MotionSLAM_ws  ·  branch dev  ·  HEAD = 20cec8e        │
│  + 工作区未 commit 改动（物模型 / 文档对齐，见 §4）           │
└───────────────┬─────────────────────────┬───────────────────┘
                │                         │
    origin (GitLab)                 ~/MotionSLAM_ws_github
    gitlab-go2:.../go2_slam.git     → github.com/zxinyu-robot/MotionSLAM_ws
    dev @ a465653（落后 10）         main @ 374d7ff+（脱敏 squash 快照）
    现场集成 · 完整历史               私人备份 · 非狗端 origin
```

| 维度 | GitLab `origin/dev` | GitHub `main` |
|------|---------------------|---------------|
| 角色 | 狗端唯一 `origin`，团队协作 | 私人脱敏归档 |
| 历史 | 线性 commit（应有 10 步，见 §2） | 2～3 个 squash commit |
| 敏感信息 | 历史 commit 含真 IP（如 `a39e7ef`） | 已脱敏占位符 |
| 同步命令 | `git push origin dev` | 在 `_github` 目录导出后 `git push origin main` |

**禁止**：把狗端 `origin` 改成 GitHub。

---

## 2. GitLab 集成线：未 push 的 10 个 commit

**本地 HEAD**：`20cec8e`（2026-08-19）  
**GitLab 远端 `origin/dev`**：`a465653` — *离线权威地图+跨分片过滤+统一投影*  
**差距**：本地超前 **10 commits**，合计约 **258 文件，+24,087 / −4,411 行**

### 2.1 演进时间线（按时间正序）

| # | SHA | 日期 | 主题 | 规模 | 要点 |
|---|-----|------|------|------|------|
| 1 | `2abc6e4` | 07-24 | Hybrid 起步 | 63 files +4698 | Super-LIO + SCAN 局部 + Nav2 全局；`hybrid_nav_stack`、`PoseGraphBackend`、大量 hybrid 脚本 |
| 2 | `a0fb446` | 07-27 | Hybrid Nav2 BT 生产主线 | 87 files +9535 | Nav2+ScanFollowPath BT；**新增 `motionslam_ipc`**；`MapSessionManager`；**证据汇报 4 份首次入库**；证据采集脚本 |
| 3 | `b0997a6` | 08-05 | **弃 Hybrid → Demo SCAN 零先验** | 184 files 净重组 | 删 Nav2 BT 插件/WaypointFsm 等；`demo_scan_stack`、`demo_scan_bt_orchestrator`；scripts/docs 大重组 |
| 4 | `2e45eb5` | 08-06 | 执行链 + 边侧上行 | 28 files +3372 | `plan_fail_recovery`、`glass_suspect_layer`；`subgraph_snapshot`、`edge_uplink` 骨架 |
| 5 | `63777de` | 08-06 | MotionCommand 解耦 | 12 files +86 | **新增 `motionslam_msgs/MotionCommand.msg`**；forwarder 改订阅 `/motion/command` |
| 6 | `19e43b3` | 08-07 | MVPI1 Step 1/2 | 50 files +4575 | lean baseline + 语义导航；`action_group`、`command_executor`、`task_context`、`execution_feedback*` |
| 7 | `18c30d7` | 08-07 | Step3 SEARCH 端侧 | 10 files +647 | `frontier_export`、`frontier_scorer`；BT 探索状态机 |
| 8 | `cfff81e` | 08-07 | Step3 边侧 mock | 8 files +764 | `edge_mvpi1_planner`、`verify_step3_*`；SEARCH→NAV handoff |
| 9 | `a39e7ef` | 08-07 | 现场 edge_host | 6 files +64 | `edge_uplink.yaml` 写 `${EDGE_HOST}` 前身（真 IP）；RGB import 修复 |
| 10 | `20cec8e` | 08-19 | **PCT C++ 常驻主链** | 80 files +4665 | `PctResidentPlanner.cpp`；PCT/SCAN submodule 指针；`return_home`；文档换 Demo1 三步 + 端侧架构 |

### 2.2 架构转折点

```text
07-24～07-27  Hybrid（Nav2 全局 + SCAN 局部 + Super-LIO）
       ↓ b0997a6 大删重组
08-05 起      Demo SCAN 零先验（BT 写 /initial_path → SCAN navi_mode=3）
       ↓ 20cec8e
08-19 现网    PCT C++ 常驻粗引导 + SCAN 局部 + BT + forwarder → Sport
```

**日常主链**（已板载验收 1.0 m / 0.43 m）：

```text
Super-LIO → pct_resident_planner → BT /initial_path → SCAN navi_mode=3
         → closed_loop → /motion/command → cmd_vel_forwarder → Sport
```

### 2.3 Submodule 指针（commit `20cec8e` 记录）

| 路径 | commit | 说明 |
|------|--------|------|
| `src/Super-LIO` | `25ff025` | branch ros2 |
| `src/livox_ros_driver2` | `13eb05e` | v1.2.6 |
| `src/unitree_ros2` | `668d1ec` | v0.3.0-19 |
| `src/SCAN-Planner` | `d62de08` | ros2-community + **本地 patch**（见 `patches/`） |
| `src/PCT-Planner` | `0cf4827` | main + **本地 patch**（可选 Python 对照） |

**工作区 submodule 本地 diff（未 commit 到 submodule 内）**：

- SCAN-Planner：8 files，约 +1930 / −1642
- PCT-Planner：6 files，约 +789 / −124

---

## 3. GitHub 备份线 commit 记录

| SHA | 日期 | 说明 |
|-----|------|------|
| `f74114f` | 08-19 | 脱敏 squash：`git archive HEAD` + 手动补未提交物模型/文档；含 `THIRD_PARTY.md`、`patches/` |
| `374d7ff` | 08-19 | 补 `docs/证据汇报/` 4 份（脱敏版） |
| （本文档） | 08-19 | 开发线 Git 记录与工作区快照 |

GitHub **不含** §2 的 10 步 commit 历史；文件内容大致对齐 `20cec8e` + §4 中已拷贝进备份的文件。

---

## 4. 工作区现状（`dev` @ `20cec8e`，未 commit）

> 以下改动**只在狗端磁盘**；GitLab `origin/dev` 与 GitHub 的 git 历史**均未记录**这些 commit。  
> GitHub `f74114f` 备份时曾**手动覆盖**部分文件内容，故 GitHub 上可能有文件副本但 dev 仍显示 modified。

### 4.1 已修改（tracked，18 paths，约 +848 / −239）

**文档（对齐物模型 / Demo1 / RoboOS 对照）**

| 文件 | 变更概要 |
|------|----------|
| `docs/架构/架构_端侧系统架构.md` | +350 行量级；三空间 OS、RoboOS 对照、RT-1～RT-4 |
| `docs/规划/项目规划_Demo1三步.md` | 五条 MVP 产品线、2-1b/c、OS 运行时 |
| `docs/项目规范/边端协同接口规范.md` | 物模型信封 `thing_envelope.v1`、禁止 cmd_vel |
| `docs/测试验收/测试记录.md` | DEV-6a 等记录 |
| `docs/开发参考/工程开发附录.md` | 小幅更新 |
| `readme.md` | 开发参考表 |

**代码（物模型下行接入）**

| 文件 | 变更概要 |
|------|----------|
| `command_executor_node.py` | +105 行；对接 `behavior_mode` 抽出 Navigate xyz |
| `directive_receiver_node.py` | +45 行；`:9879` 物模型校验 |
| `execution_feedback.py` / `execution_feedback_node.py` | 可选 `mode_id` / `mode_state` |
| `task_context.py` | `local_robot_id`、`local_platform` |
| `data_layer_types.py` | 层定义扩展 |
| `CMakeLists.txt` | 安装新脚本/测试 |
| `export_arch_diagram_png.py` | 架构图导出 |

**Submodule 指针脏状态**：`SCAN-Planner`、`PCT-Planner`（工作区有未提交 patch）

### 4.2 未跟踪（untracked，10 paths）

| 路径 | 说明 | GitHub 备份 |
|------|------|-------------|
| `behavior_mode.py` | 物模型七块 schema + 校验；拒 cmd_vel | ✅ 已在 `f74114f` |
| `test_behavior_mode.py` | 15 条单测 | ✅ 已在 `f74114f` |
| `send_thing_envelope.py` | 离线发 `:9879` 样例 | ✅ 已在 `f74114f` |
| `thing_envelope_qwen_navigate.json` | Qwen 决策 JSON 样例 | ✅ 已在 `f74114f` |
| `边侧Qwen2.5-VL接入说明.md` | 边侧 VL 接入 | ✅ 已在 `f74114f` |
| `私人GitHub备份清单.md` | 备份切分规范 | ✅ 已在 `f74114f` |
| `docs/架构/assets/架构_三空间.png` | 架构图 | ✅ 已在 `f74114f` |
| `docs/_inbox/` | 架构图 mermaid 草稿 | ✅ 已纳入 GitHub 备份 |
| `pct_planner_ws/` | Python PCT 调试 overlay | ❌ 故意不传 |

### 4.3 物模型增量摘要（未 commit 的核心）

- **Schema**：`motionslam.thing_envelope.v1`，七块 profile/properties/services/policy/models/events/world_fragments
- **下行**：`behavior_mode.execute` @ TCP `:9879`；禁止 `cmd_vel` / `/initial_path` 等直控键
- **上行**：`behavior_mode.progress|event` @ `:9880`（`build_progress_envelope` 尚未接 feedback 节点）
- **单测**：`python3 -m pytest src/motionslam_bringup/test/test_behavior_mode.py`（15 passed）

---

## 5. 三端同步矩阵（快照时刻）

| 内容 | 狗端磁盘 | GitLab `origin/dev` | GitHub `main` |
|------|----------|---------------------|---------------|
| Hybrid 历史 commit | ✅ 历史 | ❌ 缺 10 commits | ❌ 无历史 |
| PCT C++ 常驻 @ `20cec8e` | ✅ | ❌ 未 push | ✅ squash 在 `f74114f` |
| 物模型 `behavior_mode.py` | ✅ 未 commit | ❌ | ✅ 文件在、无 dev 历史 |
| 证据汇报 | ✅ @ `20cec8e`（可能含真 IP） | ❌ 未 push | ✅ `374d7ff` 脱敏版 |
| SCAN/PCT patch | ✅ 工作区 + `patches/` | ❌ | ✅ `patches/` |
| `docs/_inbox/` 草稿 | ✅ | ❌ | ❌ |
| 真机 MID360 配置 | ✅ 本地 | ❌ 未 push | ❌ 仅 example |

---

## 6. 建议后续动作

1. **GitLab**：在狗端 `git push origin dev`，补全 10 个 commit（注意历史含真 IP，私有仓）。
2. **狗端 dev**：将 §4 物模型/文档对齐 **commit 到 dev**（否则只有磁盘 + GitHub 副本，无集成线历史）。
3. **GitHub**：物模型或文档再变时，在 `MotionSLAM_ws_github` 重新导出脱敏后 push。
4. **Submodule**：SCAN/PCT 改动稳定后，更新 `patches/*.patch` 与 `THIRD_PARTY.md`。

---

## 7. 相关文档

| 文档 | 路径 |
|------|------|
| 项目规划 | [项目规划_Demo1三步.md](../规划/项目规划_Demo1三步.md) |
| 端侧架构 | [架构_端侧系统架构.md](../架构/架构_端侧系统架构.md) |
| 边端接口 | [边端协同接口规范.md](../项目规范/边端协同接口规范.md) |
| Qwen 接入 | [边侧Qwen2.5-VL接入说明.md](./边侧Qwen2.5-VL接入说明.md) |
| 备份切分 | [私人GitHub备份清单.md](./私人GitHub备份清单.md) |
| 上游 SHA | [THIRD_PARTY.md](../../THIRD_PARTY.md) |

---

*本文档由开发线快照自动生成意图整理；随 push 更新时请改「快照时间」与 commit SHA。*
