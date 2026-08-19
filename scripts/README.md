# scripts/ 目录 — 按功能分类

**唯一推荐入口：`./scripts/motionslam`**

根目录仅保留上述入口 + 本 README；实现均在子目录。`verify.py` 若存在则转发到 `verify/`。

## 目录结构

```
scripts/
├── motionslam              # 统一 CLI
├── nav/                    # 导航启停
│   ├── start_demo_scan_nav.sh        # motionslam nav start
│   ├── stop_nav.sh
│   ├── start_nav.sh              # REFUSED（历史 Nav2）
│   ├── start_hybrid_nav.sh       # REFUSED
│   └── start_scan_planner_sim.sh
│
├── map/                    # 建图与地图
│   ├── start_mapping.sh
│   ├── start_mapping_walk.sh
│   ├── ensure_nav_maps.sh
│   ├── stop_unitree_slam.sh
│   └── tools/
│
├── verify/                 # 验收与标定（F 系列 / 历史 lean 脚本仍保留）
│   ├── verify_f_series.sh
│   ├── verify_scan_planner_h5.py
│   ├── verify_step1_lean_baseline.sh # 历史 Step1，非日常入口
│   └── calibrate_odom_robo.py
│
├── dev/                    # 编译与环境
│   ├── build_ws.sh
│   ├── build_pct_planner.sh          # 仅 Python PCT 对照
│   ├── clone_pct_planner.sh
│   └── source_ws_env.sh
│
├── ops/                    # 运维
│   ├── export_go2_evidence.sh
│   └── fix_system_time.sh
│
├── offline/                # 边端 mock
├── lib/
└── legacy/                 # 历史 Nav2（symlink: legacy_nav2/）
```

已删除：`start_demo_scan_nav_lean.sh`、`start_demo_scan_nav_semantic.sh`。无 `nav start-lean`。

## 日常命令

| 功能 | 命令 |
|------|------|
| 起容器 + 起栈 | `./scripts/motionslam nav start` |
| 异常日志 | `./scripts/motionslam nav watch` / `nav watch -f` |
| 停栈 | `./scripts/motionslam nav stop` |
| 建图 | `./scripts/motionslam map start` |
| 生成 Nav 地图 | `./scripts/motionslam map ensure` |
| 编译 | `./scripts/motionslam build` |
| 证据导出 | `./scripts/motionslam export --with-maps` |

## 专项验收（非日常）

| 功能 | 命令 | 说明 |
|------|------|------|
| F 系列 | `./scripts/motionslam verify f-series` | 前段 LIO/点云；非 PCT 闭环 |
| E2E 冒烟 | `./scripts/motionslam verify smoke` | 同上 |
| SCAN H5 | `./scripts/motionslam verify h5 --distance 2` | 需已开栈 |
| 历史 Step1/2/3 | `verify step1` / `step2` / `step3` | lean / semantic / search 脚本 |

Nav2 H5、`legacy_nav2/` 不是当前主链。

## 直接调用（等价）

```bash
./scripts/motionslam nav start
./scripts/motionslam nav watch
./scripts/motionslam nav stop
```
