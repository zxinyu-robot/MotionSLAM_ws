# Go2 证据包 One-pager（内部汇报）

> 更新：2026-08-19 | 狗端 `~/MotionSLAM_ws` | 基线：**PCT C++ 常驻 + SCAN 局部 + BT 编排**

---

## 已验证（可写入主结论）

| 项 | 结果 | 证据 |
|----|------|------|
| lean SCAN 闭环 H5 2 m | PASS | 2026-08-07（历史底座，不经 PCT） |
| 边端定向 2 m（不经 PCT） | **10/10** | 2026-08-07 |
| lean 连续运行 | ≥10 min 无 OOM | Step 1 验收 |
| Gate0 PCT 与 BT 同栈、PCT 不直写执行 | PASS | 2026-08-12 干跑 |
| **板载 PCT 短程闭环** | **PASS** | 2026-08-19：1.0 m，偏差 0.43 m。`LIO→pct_resident_planner→BT /initial_path→SCAN navi_mode=3→forwarder` |

**stack**：`demo_scan`（Super-LIO mapping + C++ PCT + BT + SCAN + forwarder → Sport）  
事实表：[测试记录.md](../测试验收/测试记录.md)

---

## 正在验证（不可入主结论）

| 项 | 状态 |
|----|------|
| SAFE-1 面包屑倒序返航真机 | 代码已接；08-14 预热过，真实返航未走成 |
| 玻璃虚拟障碍真机 | 代码默认开；08-19 短程验收关了玻璃 |
| DEV-6 边端语义任务事件链 | 待复测 |
| frontier 长航时 / Token 真网络 | 待做 |
| 动态障碍 BENCH / ATE/RPE | 待填 |

---

## 尚未实现 / 未关单

- TensorRT BEV 真推理（`bev_glass_layer` 无 engine，未链 nvinfer）
- Token round-trip（DEV-7）
- 滑动地图淘汰、frontier ≥30 min
- 边端 NAV → 到达可重复（EDGE-CL）
- PolicyDB 策略归档上送

PCT→BT 接线 **已在板载链上工作**（不是 bridge 直写 goal）。缺的是边端 JSON 事件链，不是「没接线」。

---

## 复现身份（三行）

| | |
|--|--|
| **HEAD** | `dev` 工作区（含未提交的 `pct_resident_planner` 等；以现场 `git status` 为准） |
| **地图** | 无先验 mapping 模式；离线 SC-PGO 见 `sessions/` |
| **镜像** | `motionslam:humble` |

---

## 复现命令

```bash
cd ~/MotionSLAM_ws
./scripts/motionslam stop-slam
./scripts/motionslam nav start

# 08-19 短程复现
WITH_FOXGLOVE=false WITH_GLASS_AWARE=false AUTOSTART=true \
WAYPOINTS_FILE=/ws/src/motionslam_bringup/config/acceptance_short_waypoints.yaml \
./scripts/nav/start_demo_scan_nav.sh
```

不存在 `nav start-lean`。

---

## 证据包导出

```bash
./scripts/motionslam export --with-maps
```

**P0.5 补采**：[`Go2_P0.5补采runbook.md`](../运维/Go2_P0.5补采runbook.md)（脚本名仍带 nav2，**不是**当前主链验收）

完整清单：[Go2端侧证据包回传清单.md](Go2端侧证据包回传清单.md)
