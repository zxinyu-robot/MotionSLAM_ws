# 第三方依赖（迁移用）

> 本文件由 `git submodule status` 快照生成。克隆后执行 `git submodule update --init --recursive`，再 checkout 到下列 commit。

| 组件 | 远程 | 分支 | commit |
|------|------|------|--------|
| Super-LIO | https://github.com/Liansheng-Wang/Super-LIO.git | ros2 | `25ff025ef03fd203511e0232d813cf66dfe77294` |
| livox_ros_driver2 | https://github.com/Livox-SDK/livox_ros_driver2.git | — | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` |
| unitree_ros2 | https://github.com/unitreerobotics/unitree_ros2.git | — | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` |
| PCT_planner | https://github.com/VectorRobotics/PCT_planner.git | main | `0cf4827eb0e3374345b836b74ad6ea7d913b0edf` |
| SCAN-Planner | https://github.com/wuyi2121/SCAN-Planner.git | ros2-community | `d62de0844c526b9a66e4ac33733a9b8ed84943ad` |

## 本仓 patch

- `patches/scan-planner-motionslam.patch` — 应用到 SCAN-Planner 后于 `scan_planner_ws` 编译
- `patches/pct-planner-motionslam.patch` — PCT 可选对照改动

## 说明

- 本 GitHub 仓为**脱敏私人备份**，不是狗端唯一 remote。
- 雷达配置请从 `MID360_config.example.json` 复制为 `MID360_config.json` 并填现场 IP。
- 详细切分原则见 `docs/开发参考/私人GitHub备份清单.md`。
