#!/usr/bin/env bash
# Go2 端侧证据包 P0 导出（版本/文档/配置/地图 md5/日志清单）
# 用法:
#   cd ~/MotionSLAM_ws
#   ./scripts/ops/export_go2_evidence.sh              # 默认日期 export/evidence_YYYYMMDD/
#   ./scripts/ops/export_go2_evidence.sh 20260727     # 指定日期
#   ./scripts/ops/export_go2_evidence.sh --with-bags  # 额外复制 N4/N5 bag（体积大）
#   ./scripts/ops/export_go2_evidence.sh --with-maps  # 额外复制 maps/*.pcd
#
# 回传目标（开发机）:
#   /home/ubuntu/Downloads/motion_ws/logs/go2-evidence-YYYYMMDD/
set -uo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$(cd "$SCRIPTS/.." && pwd)"
cd "${WS_DIR}"

DATE="${1:-$(date +%Y%m%d)}"
if [[ "${DATE}" == "--with-bags" || "${DATE}" == "--with-maps" ]]; then
  DATE="$(date +%Y%m%d)"
fi

WITH_BAGS=0
WITH_MAPS=0
for arg in "$@"; do
  case "${arg}" in
    --with-bags) WITH_BAGS=1 ;;
    --with-maps) WITH_MAPS=1 ;;
  esac
done

EVID="${WS_DIR}/export/evidence_${DATE}"
mkdir -p "${EVID}"/{git,docker,docs,config,scripts,logs,maps,bags,hybrid,snapshots}

echo "[export] 证据包目录: ${EVID}"

# ---------- 0. Git / Docker 身份 ----------
git log -5 --oneline > "${EVID}/git/log_oneline.txt" 2>/dev/null || true
git status -sb > "${EVID}/git/status_sb.txt" 2>/dev/null || true
git rev-parse HEAD > "${EVID}/git/HEAD.txt" 2>/dev/null || true
git submodule status > "${EVID}/git/submodule_status.txt" 2>/dev/null || true
git diff --stat > "${EVID}/git/diff_stat.txt" 2>/dev/null || true

docker images motionslam:humble > "${EVID}/docker/images_motionslam_humble.txt" 2>/dev/null || true
docker inspect motionslam:humble --format='ID={{.Id}} Created={{.Created}}' \
  > "${EVID}/docker/image_inspect.txt" 2>/dev/null || true
cp -a docker/Dockerfile docker/run_container.sh docker/cyclonedds.xml "${EVID}/docker/" 2>/dev/null || true

# ---------- 1. versions.txt ----------
{
  echo "=== host ==="
  uname -a
  echo
  cat /etc/os-release 2>/dev/null || true
  echo
  free -h
  echo
  df -h /
  echo
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi -L 2>/dev/null || nvidia-smi 2>/dev/null || true
  elif [[ -r /proc/device-tree/model ]]; then
    echo -n "device_model: "
    tr -d '\0' < /proc/device-tree/model
    echo
  fi
  echo
  python3 --version 2>/dev/null || true
  echo
  echo "=== docker (motionslam) ==="
  if docker ps --format '{{.Names}}' | grep -qx motionslam; then
    docker exec motionslam bash -lc 'echo ROS_DISTRO=$ROS_DISTRO; dpkg -l ros-humble-ros-base 2>/dev/null | tail -1; echo RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID; echo CYCLONEDDS_URI=$CYCLONEDDS_URI' \
      2>/dev/null || echo "WARN: 容器 motionslam 未运行"
  else
    echo "WARN: 容器 motionslam 不存在"
  fi
} > "${EVID}/versions.txt"

cp readme.md "${EVID}/readme.md" 2>/dev/null || true

# ---------- 2. 文档（纯 Nav2 主证据） ----------
DOCS=(
  docs/测试验收/测试记录.md
  docs/运维/Demo阶段3D_runbook.md
  docs/运维/runbook.md
  docs/测试验收/调试记录.md
  docs/证据汇报/Go2现场采集清单.md
  docs/证据汇报/Go2端侧证据包回传清单.md
  docs/证据汇报/Go2证据包One-pager.md
  docs/证据汇报/Go2专利交底-架构与差异原材料.md
)
for f in "${DOCS[@]}"; do
  [[ -f "${f}" ]] && cp -a "${f}" "${EVID}/docs/" || echo "MISSING: ${f}" >> "${EVID}/INVENTORY_MISSING.txt"
done

# N1/N4/N5/N6 相关 log
LOG_PATTERNS=(
  'nav_20260721*'
  'nav_20260723*n6*'
  'nav_section6*'
  'nav_20260723*'
)
cp_safe() {
  cp --no-preserve=ownership,mode "$@" 2>/dev/null || cp "$@" 2>/dev/null || true
}

for pat in "${LOG_PATTERNS[@]}"; do
  shopt -s nullglob
  for f in logs/${pat}; do
    cp_safe "${f}" "${EVID}/logs/"
  done
  shopt -u nullglob
done

# N6 verify 原始输出（从 log 提取或复制）
if [[ -f logs/nav_20260723_n6_h5_3m.log ]]; then
  cp_safe logs/nav_20260723_n6_h5_3m.log "${EVID}/logs/verify_h5_n6_raw.txt"
fi

# ---------- 3. bags 清单 / 可选复制 ----------
{
  echo "# bags 目录清单 $(date -Iseconds)"
  ls -lah bags/ 2>/dev/null || true
} > "${EVID}/bags/MANIFEST.txt"

BAG_DIRS=(
  20260721_N4_2m_r5_b53c2b6
  20260721_N5_estop_walking_b53c2b6
  20260721_N5_estop_pass_b53c2b6
  20260721_N4_2m_fail_b53c2b6
  20260721_N4_2m_r2_fail_b53c2b6
  20260721_N4_2m_r3_fail_b53c2b6
  20260721_N4_2m_r4_fail_b53c2b6
)
for d in "${BAG_DIRS[@]}"; do
  if [[ -d "bags/${d}" ]]; then
    du -sh "bags/${d}" >> "${EVID}/bags/MANIFEST.txt"
    if [[ "${WITH_BAGS}" -eq 1 ]]; then
      cp -a "bags/${d}" "${EVID}/bags/"
    fi
  else
    echo "MISSING bag dir: bags/${d}" >> "${EVID}/INVENTORY_MISSING.txt"
  fi
done
if [[ -f bags/20260721_go2_session.tar.gz ]]; then
  ls -lh bags/20260721_go2_session.tar.gz >> "${EVID}/bags/MANIFEST.txt"
  [[ "${WITH_BAGS}" -eq 1 ]] && cp -a bags/20260721_go2_session.tar.gz "${EVID}/bags/"
fi
echo "N6: 仅有 log，无 bag（见 logs/verify_h5_n6_raw.txt）" >> "${EVID}/bags/MANIFEST.txt"

# ---------- 4. 地图 md5 ----------
if ls maps/map_*.pcd >/dev/null 2>&1; then
  md5sum maps/map_*.pcd > "${EVID}/maps/maps_md5.txt"
  [[ -f maps/map.pcd ]] && md5sum maps/map.pcd >> "${EVID}/maps/maps_md5.txt"
  if [[ "${WITH_MAPS}" -eq 1 ]]; then
    cp -a maps/map_reloc.pcd maps/map_nav.pcd maps/map_viz.pcd maps/map.pcd "${EVID}/maps/" 2>/dev/null || true
  fi
else
  echo "MISSING: maps/map_*.pcd" >> "${EVID}/INVENTORY_MISSING.txt"
fi

# ---------- 5. 配置与 launch（Demo） ----------
CONFIGS=(
  src/motionslam_bringup/config/demo_scan_planner.yaml
  src/motionslam_bringup/config/demo_scan_waypoints.yaml
  src/motionslam_bringup/config/pipeline.yaml
  src/motionslam_bringup/config/super_lio_mid360.yaml
  src/motionslam_bringup/launch/demo_scan_stack.launch.py
)
for f in "${CONFIGS[@]}"; do
  [[ -f "${f}" ]] && cp -a "${f}" "${EVID}/config/" || echo "MISSING: ${f}" >> "${EVID}/INVENTORY_MISSING.txt"
done

SCRIPTS=(
  scripts/nav/start_demo_scan_nav.sh
  scripts/nav/stop_nav.sh
  scripts/map/stop_unitree_slam.sh
  scripts/verify/verify_h5_arrival.py
  scripts/verify/verify_f3_demo_cloud.py
  scripts/verify/verify_f4_demo_scan.py
  scripts/verify/h5_arrival_utils.py
)
for f in "${SCRIPTS[@]}"; do
  [[ -f "${f}" ]] && cp -a "${f}" "${EVID}/scripts/" || echo "MISSING: ${f}" >> "${EVID}/INVENTORY_MISSING.txt"
done

# ---------- 7. Hybrid 对照（feature/hybrid-baseline，dev 不打包） ----------
echo "STATUS: Hybrid 栈在 feature/hybrid-baseline 分支维护，dev 证据包不含 hybrid 文件" > "${EVID}/hybrid/README_FEATURE_BRANCH.txt"

# ---------- META.txt 预填 ----------
GIT_HEAD="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
GIT_DIRTY="no"
git diff --quiet 2>/dev/null || GIT_DIRTY="yes"
git diff --cached --quiet 2>/dev/null || GIT_DIRTY="yes"

MAP_MD5=""
if [[ -f "${EVID}/maps/maps_md5.txt" ]]; then
  MAP_MD5="$(awk '{print $1}' "${EVID}/maps/maps_md5.txt" | paste -sd'/' -)"
fi

DOCKER_ID="$(docker inspect motionslam:humble --format='{{.Id}}' 2>/dev/null | cut -c8-19 || echo unknown)"
DOCKER_CREATED="$(docker inspect motionslam:humble --format='{{.Created}}' 2>/dev/null || echo unknown)"

SUBMOD_FILE="${EVID}/git/submodule_status.txt"
SUB_SUPER="$(grep Super-LIO "${SUBMOD_FILE}" 2>/dev/null | awk '{print $1,$2}' || echo unknown)"
SUB_LIVOX="$(grep livox "${SUBMOD_FILE}" 2>/dev/null | awk '{print $1,$2}' || echo unknown)"
SUB_UNITREE="$(grep unitree_ros2 "${SUBMOD_FILE}" 2>/dev/null | awk '{print $1,$2}' || echo unknown)"

cat > "${EVID}/META.txt" <<EOF
date_time=$(date -Iseconds)
operator=
stack=pure_nav2
git_head=${GIT_HEAD}
git_branch=${GIT_BRANCH}
git_dirty=${GIT_DIRTY}
submodule_super_lio=${SUB_SUPER}
submodule_livox=${SUB_LIVOX}
submodule_unitree_ros2=${SUB_UNITREE}
docker_image=motionslam:humble
docker_id=${DOCKER_ID}
docker_created=${DOCKER_CREATED}
map_set=${MAP_MD5}
launch_cmd=./scripts/motionslam nav start
verify_cmd=./scripts/nav/stop_nav.sh ; ros2 topic echo /demo/mission/event
result=PARTIAL
notes=P0 静态导出；Demo 主线；Hybrid 见 feature/hybrid-baseline
EOF

# ---------- 验收清单状态 ----------
cat > "${EVID}/CHECKLIST_STATUS.md" <<'EOF'
# 证据包验收门槛（开发机收到后核对）

| 项 | 状态 | 说明 |
|----|------|------|
| HEAD + submodule + dirty | 见 git/ | |
| 测试记录原文 | docs/测试验收/测试记录.md | |
| N4 PASS bag | bags/20260721_N4_2m_r5_b53c2b6 | 默认仅 MANIFEST，加 --with-bags 复制 |
| N5 PASS bag | bags/20260721_N5_estop_walking_b53c2b6 | 同上 |
| N6 PASS log | logs/verify_h5_n6_raw.txt | **无 bag** |
| map_reloc/nav/viz + md5 | maps/maps_md5.txt | 加 --with-maps 复制 pcd |
| demo 配置 + start/stop | config/ scripts/ | |
| topic/node/TF 快照 | snapshots/ | **待 Nav2 运行中补采** |
| META.txt | META.txt | 补 operator / result |

缺任一项 → 证据闭环仍为 **部分通过**，不可升为完整通过。
EOF

echo "[export] 完成: ${EVID}"
echo "[export] 回传示例: rsync -avz ${EVID}/ ubuntu@<dev>:/home/ubuntu/Downloads/motion_ws/logs/go2-evidence_${DATE}/"
du -sh "${EVID}"
