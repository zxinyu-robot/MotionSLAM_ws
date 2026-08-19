#!/usr/bin/env bash
# 打包 SC-PGO Session：OctVox + keyframes + edges + manifest（任务结束/回基站后）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${1:-${ROOT}/sessions/sc_pgo_$(date +%Y%m%d_%H%M%S)}"
MAP_DIR="${SUPER_LIO_MAP_DIR:-${ROOT}/map}"
SESSION_TMP="${SESSION_EXPORT_DIR:-/tmp/motionslam_session}"

mkdir -p "${OUT}/maps/PCD" "${OUT}/keyframes" "${OUT}/edges" "${OUT}/calibration"

echo "=== SC-PGO Session 导出 → ${OUT} ==="

if [[ -f "${MAP_DIR}/OctVoxMap.pcd" ]]; then
  cp -a "${MAP_DIR}/OctVoxMap.pcd" "${OUT}/maps/"
  echo "  OctVoxMap.pcd"
fi

if [[ -d "${MAP_DIR}/PCD" ]]; then
  cp -a "${MAP_DIR}/PCD/"*.pcd "${OUT}/maps/PCD/" 2>/dev/null || true
  echo "  PCD/scans_*.pcd ($(ls -1 "${OUT}/maps/PCD/" 2>/dev/null | wc -l) files)"
fi

if [[ -d "${SESSION_TMP}/keyframes" ]]; then
  cp -a "${SESSION_TMP}/keyframes/"* "${OUT}/keyframes/" 2>/dev/null || true
fi

if [[ -f /tmp/T_map_world_final.yaml ]]; then
  cp /tmp/T_map_world_final.yaml "${OUT}/calibration/"
fi

cat > "${OUT}/session_manifest.yaml" <<EOF
schema_version: 1
mode: ONLINE_EXPORT
map_dir: ${MAP_DIR}
session_tmp: ${SESSION_TMP}
notes: |
  边侧运行: python3 scripts/offline/run_sc_pgo_offline.py --session ${OUT}
EOF

echo "OK: ${OUT}/session_manifest.yaml"
echo "下一步: python3 scripts/offline/run_sc_pgo_offline.py --session ${OUT}"
