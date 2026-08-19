#!/usr/bin/env bash
# 从 maps/PCD/scans_*.pcd 生成 map_reloc / map_viz / map_nav（Nav2 + 重定位必需）
# 用法:
#   ./scripts/map/ensure_nav_maps.sh           # 缺则生成
#   ./scripts/map/ensure_nav_maps.sh --force   # 强制重建 (原 post_mapping_maps.sh)
set -euo pipefail
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
cd "$ROOT"
MAPS="${MOTIONSLAM_MAPS:-$ROOT/maps}"
FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

need=$FORCE
if [[ "$need" -eq 0 ]]; then
  for f in map_reloc.pcd map_nav.pcd; do
    if [[ ! -f "$MAPS/$f" ]]; then
      need=1
    fi
  done
fi
if [[ "$need" -eq 0 ]]; then
  echo "OK: 已有 $MAPS/map_reloc.pcd 与 map_nav.pcd"
  exit 0
fi

if ! compgen -G "$MAPS/PCD/scans_*.pcd" >/dev/null && [[ ! -f "$MAPS/map.pcd" ]]; then
  echo "ERROR: 无 $MAPS/PCD/scans_*.pcd 或 map.pcd，请先建图" >&2
  exit 1
fi

DOWN="$SCRIPTS/map/tools/downsample_reloc_map.py"
echo "生成 Nav/Reloc 地图 → $MAPS ..."
MF="${MIN_FRAGMENTS:-2}"
python3 "$DOWN" \
  --leaf 0.5 --min-fragments "$MF" --min-neighbors 1 \
  --output "$MAPS/map_reloc.pcd"
python3 "$DOWN" \
  --leaf 0.2 --min-fragments 1 --output "$MAPS/map_viz.pcd"
python3 "$DOWN" \
  --leaf 0.25 --min-fragments "$MF" --min-neighbors 1 \
  --output "$MAPS/map_nav.pcd"
echo "OK: map_reloc.pcd map_viz.pcd map_nav.pcd"
echo "认位: python3 scripts/map/tools/save_reloc_spot.py --name <工位> --set-default"
echo "导航: ./scripts/motionslam nav start"
