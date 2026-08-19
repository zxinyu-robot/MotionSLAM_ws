# Go2 端侧 → 统筹仓 证据包回传清单

> **基线（dev）**：PCT C++ 常驻 + SCAN 局部 + BT → `./scripts/motionslam nav start`  
> 架构：[架构_端侧系统架构.md](../架构/架构_端侧系统架构.md) · 验收事实：[测试记录.md](../测试验收/测试记录.md)

---

## 0. 打包约定

证据包目录：`~/MotionSLAM_ws/export/evidence_YYYYMMDD/`

每轮实验填写 `META.txt`（模板：`export/META.txt.template`）。必填字段：

| 字段 | 说明 |
|------|------|
| date_time | ISO 时间 |
| operator | 操作人 |
| stack | `demo_scan`（默认 PCT C++） |
| git_head / git_branch / git_dirty | 复现身份 |
| submodule_* | Super-LIO / livox / unitree_ros2 |
| docker_image / docker_id / docker_created | 默认 `motionslam:humble` |
| map_set | map_reloc / map_nav / map_viz 的 md5 |
| launch_cmd / verify_cmd | 启动与验收命令 |
| result | PASS / FAIL / PARTIAL |
| notes | 例外说明 |

---

## 1. 【P0】版本与复现身份

### 已有 → 脚本自动导出

- `git/log_oneline.txt`、`status_sb.txt`、`HEAD.txt`、`submodule_status.txt`
- `docker/images_motionslam_humble.txt`、`Dockerfile`、`run_container.sh`、`cyclonedds.xml`
- `readme.md`
- `versions.txt`（含 host + 容器内 ros2/RMW/DOMAIN_ID）

### 现场盘点（2026-07-27 更新）

| 项 | 状态 |
|----|------|
| 证据包 | `export/evidence_20260727/` **2.6G**（含 bag + maps + snapshots） |
| HEAD | `2abc6e4`（dev，ahead 1，**dirty**） |
| N4/N5 证据 commit | `b53c2b6`（bag 目录名后缀一致） |
| 容器镜像 | `motionslam:humble`（`e8a61b4b29e9`） |
| topic/TF 快照 | ✅ `snapshots/`（2026-07-27 nav_stack 运行中采集） |

---

## 2. 【P0】主链已通过证据

> 2026-07 Nav2 时代 bag 仍保留于 `bags/`，作历史对照，**不代表当前主链**。  
> 当前主链通过项：测试记录 G0-4 / BOARD-CL（2026-08-19）。

### A. 文档（原文）

| 文件 | 状态 |
|------|------|
| docs/测试验收/测试记录.md | ✅ |
| docs/运维/Demo阶段3D_runbook.md | ✅ |
| docs/运维/runbook.md | ✅ |
| docs/测试验收/调试记录.md | ✅ |
| docs/证据汇报/Go2现场采集清单.md | ✅（2026-07 历史采集；主链以测试记录为准） |
### B. Bag / 目录

| 路径 | 用例 | 状态 |
|------|------|------|
| bags/20260721_N4_2m_r5_b53c2b6 | N4 PASS | ✅ 存在 |
| bags/20260721_N5_estop_walking_b53c2b6 | N5 PASS（行走中 estop） | ✅ |
| bags/20260721_N5_estop_pass_b53c2b6 | N5 PASS | ✅ |
| bags/20260721_N4_2m_fail_b53c2b6 | N4 失败对照 | ✅ |
| bags/20260721_N4_2m_r2~r4_fail_b53c2b6 | N4 失败对照 | ✅ |
| bags/20260721_go2_session.tar.gz | 会话归档 (~735 MB) | ✅ |

默认导出仅含 `bags/MANIFEST.txt`；完整复制：`./scripts/motionslam export --with-bags`

### C. N6 证据

| 项 | 状态 |
|----|------|
| 测试记录 | **PASS**（误差 0.283 m，2.72 m，11.6 s） |
| log | `logs/nav_20260723_n6_h5_3m.log` ✅ |
| verify 原始输出 | 已复制为 `logs/verify_h5_n6_raw.txt` |
| bag | **❌ 仅有 log，无 bag** — 建议 P0.5 补录 |

### D. 地图三件套

| 文件 | 状态 |
|------|------|
| maps/map_reloc.pcd | ✅ |
| maps/map_nav.pcd | ✅ |
| maps/map_viz.pcd | ✅ |
| maps/map.pcd | ✅（会话用） |
| maps/maps_md5.txt | 脚本生成 |

### E. 配置与 launch（SCAN 主链复现）

见 `export_go2_evidence.sh` 中 CONFIGS / SCRIPTS 列表；导出至 `config/`、`scripts/`。

---

## 3. 【P0.5】关键话题与系统拓扑快照

**旁路**：`./scripts/motionslam verify step1 --record`（历史 lean 脚本）

主路径开栈（dev）：`./scripts/motionslam nav start`

---

## 4. 【P0.5 补采】定位量化 ATE/RPE

**Runbook**：[`Go2_P0.5补采runbook.md`](Go2_P0.5补采runbook.md) §4

```bash
./scripts/run_ate_rpe_capture.sh straight_3m   # 空场 3m 往返
./scripts/run_ate_rpe_capture.sh n6_3m         # 同 N6 短路径
./scripts/run_ate_rpe_capture.sh static_60s     # 静止 60s 漂移
```

每 bag 自动生成 `META_ATE_*.txt`（含 ground_truth=无外部真值）。

---

## 5. 【P0.5 补采】静态障碍（纸箱）

**Runbook**：[`Go2_P0.5补采runbook.md`](Go2_P0.5补采runbook.md) §2

待当前主链专用障碍验收；现场以 Foxglove `/planning/bspline`、`/initial_path` 与 `/demo/mission/event` 记录。`run_nav2_obstacle_capture.sh` 是历史命名，**不要**当成 Nav2 主链仍在。

---

## 6. 【P0.5 补采】NAV-4 WiFi 中断 / 本地降级

**Runbook**：[`Go2_P0.5补采runbook.md`](Go2_P0.5补采runbook.md) §3

```bash
DEV_IP=${DEV_IP} ./scripts/run_nav4_wifi_disconnect_capture.sh
# 或手动断网（不设 DEV_IP）
```

---

## 6b. N6 bag 补录

```bash
./scripts/run_n6_bag_capture.sh
```

---

## 7. 【已通过板载 / 待边端】PCT + SCAN

**板载短程（可写入主结论）**：2026-08-19 BOARD-CL。日志 `logs/demo_scan_nav_20260819_143445.log`。

**不得**写入主结论，直至对应验收关闭：

- 边端语义指令到达（EDGE-CL / DEV-6）
- 真机返航（SAFE-1）
- Token 真网络（DEV-7）

---

## 8. 专利交底专用

见 [Go2专利交底-架构与差异原材料.md](Go2专利交底-架构与差异原材料.md)

---

## 9. 论文安全版 outline

**可公开/可脱敏**：

- 平台：Unitree GO2 + Mid-360 + NX 边端 + Docker Humble
- N4：2 m，误差 **0.253 m**，7.3 s
- N6：3 m，误差 **0.283 m**，11.6 s
- N5：estop 后 3 s 位移 **0.011 m**
- 场景：室内空场、无先验 mapping、**PCT C++ + SCAN 局部到达（08-19 1.0 m / 0.43 m）**
- 历史 N4/N5/N6 数字仅作 07 月 lean/Nav2 对照，不替代 BOARD-CL

**勿写入对外稿**：

- 完整 Token schema、双触发伪码、未申请独权细节
- 「PCT 未接线 / Step 5 未验收」（与 08-19 板载事实不符）
- 边端语义闭环、返航、Token 尚未关单的能力写成已交付

---

## 10. 内部汇报 One-pager

见 [Go2证据包One-pager.md](Go2证据包One-pager.md)

---

## 11. 执行优先级

| 优先级 | 项 |
|--------|-----|
| **P0 今天** | 导出版本身份 + 测试记录 + 配置；BOARD-CL 08-19 日志 |
| **P0.5 本周** | ATE/RPE bag；静态障碍 SCAN 复测；SAFE-1 返航；topic/TF 快照 |
| **P1** | DEV-6 边端事件链；CPU/内存/带宽 |

---

## 12. 回传验收门槛

```
[ ] HEAD + submodule + dirty
[ ] BOARD-CL 08-19 日志 / 测试记录原文
[x] N4 PASS bag（历史 Nav2/lean 对照）
[x] N5 PASS bag（历史）
[ ] N6 PASS log（最好有 bag）          ← 当前缺 bag
[x] map_reloc / map_nav / map_viz + md5
[x] demo_scan_planner.yaml + start/stop 脚本
[x] topic/node/TF 快照                 ← frames.pdf 缺，有 TF_FRAMES.yaml
[ ] META.txt 填全（operator）
```

缺任一项 → **不可**将「证据闭环」从部分通过升为完整通过。

---

## 回传命令示例

```bash
cd ~/MotionSLAM_ws
./scripts/motionslam export --with-maps
# 可选大文件:
# ./scripts/motionslam export --with-bags --with-maps

rsync -avz --progress \
  export/evidence_$(date +%Y%m%d)/ \
  ubuntu@<开发机IP>:/home/ubuntu/Downloads/motion_ws/logs/go2-evidence-$(date +%Y%m%d)/
```
