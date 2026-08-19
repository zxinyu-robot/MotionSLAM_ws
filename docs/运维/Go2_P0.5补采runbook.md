# Go2 P0.5 补采 Runbook

> **基线（dev）**：`./scripts/motionslam nav start`（默认 PCT C++ + SCAN + BT）  
> **证据目录**：`bags/`、`logs/`、`export/evidence_YYYYMMDD/META_*.txt`  
> 下列 `run_nav2_*` / N6 H5 脚本是历史采集名，**不代表 Nav2 仍是主链**。当前到达证据以 [测试记录.md](../测试验收/测试记录.md) BOARD-CL 为准。

---

## 0. 前置

```bash
cd ~/MotionSLAM_ws
docker start motionslam
chmod +x scripts/run_*_capture.sh scripts/lib/*.sh
./scripts/ensure_nav_maps.sh   # 或容器内等价命令
```

| 检查 | 命令 |
|------|------|
| 地图 | `md5sum maps/map_*.pcd` |
| 空场 | 3 m 内无障碍 |
| Mid-360 | ping ${LIDAR_IP} |
| forwarder | 启动参数 `with_forwarder:=true` |

---

## 1. N6 bag 补录（缺 bag）

**目的**：补齐 N6 PASS 的可回放 bag（3 m H5）。

```bash
./scripts/run_n6_bag_capture.sh
# 可选: RELOC_SPOT=map_origin DISTANCE=3 OPERATOR=张三
```

**产出**：
- `bags/YYYYMMDD_N6_3m_<git>/`
- `logs/nav_YYYYMMDD_n6_h5_3m_<ts>.log`
- `export/evidence_YYYYMMDD/META_N6_<ts>.txt`

**通过标准**：误差 < 0.3 m（与历史 0.283 m 一致）。

---

## 2. NAV-2 静态障碍（纸箱）

**目的**：途中纸箱 — 绕行 / 停住 / 失败分类。

**准备**：在狗头正前方路径 **1–2 m** 处放纸箱，高度 0.2–1.5 m。

```bash
./scripts/run_nav2_obstacle_capture.sh
# DISTANCE=3
```

**产出**：
- `bags/YYYYMMDD_NAV2_obstacle_<git>/`
- `logs/nav_*_nav2_obstacle_*.log`（含 `PASS [绕行|停住|到达]`）
- `META_NAV2_*.txt` — 填 `photo_path` / `video_path`

**判定**（脚本内置）：
| 结果 | 含义 |
|------|------|
| PASS [到达] | 空场或绕过后到达 |
| PASS [停住] | 距障静止 ≥8 s |
| PASS [绕行] | 侧偏 ≥0.35 m 且前进 |
| FAIL [试探] | 前后微动无净前进 |

---

## 3. NAV-4 WiFi / 远程链路中断

**目的**：导航中断开发机/办公网后，NX 本地继续或安全停车。

```bash
# 方式 A: 自动 iptables 阻断开发机 IP（需 sudo 免密或现场输入）
DEV_IP=${DEV_IP} DISCONNECT_AT=8 ./scripts/run_nav4_wifi_disconnect_capture.sh

# 方式 B: 手动断网（脚本 t=8s 时提示）
./scripts/run_nav4_wifi_disconnect_capture.sh
```

**产出**：
- `bags/YYYYMMDD_NAV4_wifi_<git>/`
- `logs/nav_*_nav4_disconnect_*.txt` — **断网时刻标记**
- `META_NAV4_*.txt` — `conclusion` 字段

**结论枚举**：`本地完成` | `安全停车` | `本地部分完成/未到达` | 人工判定

**断网后探针**：脚本结束调用 `estop_go2`；请现场确认 `obstacles_avoid` 仍响应。

---

## 4. ATE/RPE bag（三场景）

**目的**：离线 evo 量化；**无外部真值**，仅相对闭环/回测。

```bash
# 4.1 空场直线 3 m 往返
./scripts/run_ate_rpe_capture.sh straight_3m

# 4.2 已知地图短路径（同 N6）
./scripts/run_ate_rpe_capture.sh n6_3m

# 4.3 静止 60 s 漂移
./scripts/run_ate_rpe_capture.sh static_60s
```

**bag 话题**（默认 lite，无点云）：
- `/lio/robo/odom`, `/lio/odom`, `/tf`, `/tf_static`, `/cmd_vel`, `/plan`, costmap

含点云：`ATE_BAG_LITE=0 ./scripts/run_ate_rpe_capture.sh straight_3m`

**离线 evo 示例**（开发机）：

```bash
ros2 bag play bags/YYYYMMDD_ATE_straight_3m_*/
# 需安装: pip install evo
evo_traj bag bags/... --topic /lio/robo/odom --save_plot ate_straight.png
```

---

## 5. 补采后更新证据包

```bash
# 重新打包（含新 bag）
./scripts/export_go2_evidence.sh $(date +%Y%m%d) --with-bags --with-maps

# 若 Nav2 运行中补 TF 快照
./scripts/legacy_nav2/capture_nav_snapshot.sh export/evidence_$(date +%Y%m%d)/snapshots
```

**META.txt**：补 `operator=`、`result=PASS/FAIL/PARTIAL`。

---

## 6. 回传

```bash
rsync -avz --progress \
  ~/MotionSLAM_ws/export/evidence_$(date +%Y%m%d)/ \
  ~/MotionSLAM_ws/bags/2026*_N6_* \
  ~/MotionSLAM_ws/bags/2026*_NAV2_* \
  ~/MotionSLAM_ws/bags/2026*_NAV4_* \
  ~/MotionSLAM_ws/bags/2026*_ATE_* \
  ubuntu@<dev>:/home/ubuntu/Downloads/motion_ws/logs/go2-evidence-$(date +%Y%m%d)/
```

---

## 7. 建议执行顺序（现场）

1. `./scripts/run_n6_bag_capture.sh`
2. `./scripts/run_ate_rpe_capture.sh straight_3m`
3. `./scripts/run_ate_rpe_capture.sh static_60s`
4. 放置纸箱 → `./scripts/run_nav2_obstacle_capture.sh`
5. `./scripts/run_nav4_wifi_disconnect_capture.sh`
6. `./scripts/export_go2_evidence.sh --with-bags --with-maps`

---

## 8. 脚本索引

| 脚本 | 用途 |
|------|------|
| `run_n6_bag_capture.sh` | N6 3 m + bag |
| `run_nav2_obstacle_capture.sh` | NAV-2 纸箱 |
| `run_nav4_wifi_disconnect_capture.sh` | NAV-4 断网 |
| `run_ate_rpe_capture.sh` | ATE 三场景 |
| `verify_nav4_wifi_disconnect.py` | NAV-4 验收逻辑 |
| `verify_ate_straight_3m.py` | 往返 3 m |
| `verify_ate_static_drift.py` | 静止漂移 |
| `lib/evidence_capture.sh` | 公共：启栈/录 bag/META |
