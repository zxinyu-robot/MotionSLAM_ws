# map/tools — 建图 / 重定位 / 调试工具

| 脚本 | 用途 |
|------|------|
| `downsample_reloc_map.py` | PCD 降采样 → map_reloc / map_nav |
| `save_reloc_spot.py` | 保存认位工位 |
| `read_reloc_init_pose.py` | 读 /lio/odom 作 ICP 初值 |
| `tune_reloc_yaw.py` | 微调认位 yaw |
| `estimate_scan_map_yaw.py` | 估计地图与 scan 朝向差 |
| `verify_scene_heading.py` | Foxglove 目视朝向核对 |
| `publish_live_map_cloud.py` | 发布 live map 点云 |
| `walk_odom_distance.py` | odom 行走距离辅助 |
| `watch_mapping_backend.sh` | 监视 mapping 后端 |
| `check_foxglove_ws.sh` | Foxglove bridge 检查 |

由 `map/ensure_nav_maps.sh` 调用 `downsample_reloc_map.py`。

```bash
python3 scripts/map/tools/save_reloc_spot.py --name <工位> --set-default
# 或（symlink）: scripts/tools/save_reloc_spot.py
```
