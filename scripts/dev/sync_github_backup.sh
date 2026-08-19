#!/usr/bin/env bash
# 从狗端 ~/MotionSLAM_ws 同步最新内容到 ~/MotionSLAM_ws_github 并 commit。
# push 需临时走 WiFi（见末尾说明）。不修改 MotionSLAM_ws 的 dev 分支。
set -euo pipefail

SRC="${SRC:-$HOME/MotionSLAM_ws}"
BACKUP="${BACKUP:-$HOME/MotionSLAM_ws_github}"

if [[ ! -d "$SRC/.git" ]]; then
  echo "missing $SRC/.git" >&2
  exit 1
fi
if [[ ! -d "$BACKUP/.git" ]]; then
  echo "missing $BACKUP/.git — clone or init GitHub backup first" >&2
  exit 1
fi

SYNC_PATHS=(
  docs/开发参考/工程开发附录.md
  docs/架构/架构_端侧系统架构.md
  docs/测试验收/测试记录.md
  docs/规划/项目规划_Demo1三步.md
  docs/项目规范/边端协同接口规范.md
  readme.md
  scripts/dev/export_arch_diagram_png.py
  src/motionslam_bringup/CMakeLists.txt
  src/motionslam_bringup/scripts/behavior_mode.py
  src/motionslam_bringup/scripts/command_executor_node.py
  src/motionslam_bringup/scripts/data_layer_types.py
  src/motionslam_bringup/scripts/directive_receiver_node.py
  src/motionslam_bringup/scripts/execution_feedback.py
  src/motionslam_bringup/scripts/execution_feedback_node.py
  src/motionslam_bringup/scripts/task_context.py
  src/motionslam_bringup/test/test_behavior_mode.py
  scripts/offline/send_thing_envelope.py
  scripts/offline/thing_envelope_qwen_navigate.json
  docs/开发参考/私人GitHub备份清单.md
  docs/开发参考/边侧Qwen2.5-VL接入说明.md
  docs/开发参考/开发线Git记录与工作区快照.md
)

for f in "${SYNC_PATHS[@]}"; do
  if [[ -f "$SRC/$f" ]]; then
    mkdir -p "$BACKUP/$(dirname "$f")"
    cp "$SRC/$f" "$BACKUP/$f"
  fi
done

mkdir -p "$BACKUP/docs/架构/assets" "$BACKUP/docs/_inbox" "$BACKUP/docs/证据汇报" "$BACKUP/patches"
cp -a "$SRC/docs/_inbox/." "$BACKUP/docs/_inbox/" 2>/dev/null || true
cp "$SRC/docs/架构/assets/架构_三空间.png" "$BACKUP/docs/架构/assets/" 2>/dev/null || true
cp "$SRC/docs/证据汇报/"*.md "$BACKUP/docs/证据汇报/" 2>/dev/null || true

(cd "$SRC/src/SCAN-Planner" && git diff > "$BACKUP/patches/scan-planner-motionslam.patch") || true
(cd "$SRC/src/PCT-Planner" && git diff > "$BACKUP/patches/pct-planner-motionslam.patch") || true

# 脱敏现场网段（仅备份目录）
find "$BACKUP" -type f \( -name '*.md' -o -name '*.yaml' -o -name '*.py' -o -name '*.sh' -o -name '*.launch.py' -o -name '*.json' \) -print0 \
  | xargs -0 sed -i \
    -e 's/192\.168\.110\.26/${EDGE_HOST}/g' \
    -e 's/192\.168\.110\.124/${GO2_IP}/g' \
    -e 's/192\.168\.110\.127/${GO2_IP}/g' \
    -e 's/192\.168\.110\.61/${GO2_WLAN_IP}/g' \
    -e 's/192\.168\.110\.93/${BIND_ADDRESS}/g' \
    -e 's/192\.168\.110\.100/${DEV_IP}/g' \
    -e 's/192\.168\.110\.x/${DEV_IP}/g' \
    -e 's/192\.168\.123\.18/${LIDAR_HOST_IP}/g' \
    -e 's/192\.168\.123\.20/${LIDAR_IP}/g' \
    -e 's/192\.168\.123\.161/${GO2_ETH_IP}/g' \
    -e 's/192\.168\.123\.0\/24/${LIDAR_SUBNET}/g' \
    -e 's/192\.168\.12\.1/${GATEWAY_IP}/g'

# 恢复 GitHub 备份仓 readme 顶栏（狗端 readme 无此段）
BACKUP="$BACKUP" python3 <<'PY'
import os
from pathlib import Path
p = Path(os.environ["BACKUP"]) / "readme.md"
text = p.read_text(encoding="utf-8")
banner = (
    "> **GitHub 私人备份仓**（脱敏快照，与狗端 `~/MotionSLAM_ws` 同步）。"
    "集成仓仍以 GitLab `origin` 为准；上游见 [THIRD_PARTY.md](THIRD_PARTY.md)。  \n"
    "> 雷达配置：复制 `src/motionslam_bringup/config/MID360_config.example.json` → `MID360_config.json` 后填现场 IP。  \n"
    "> **Git / 工作区状态**：[开发线Git记录与工作区快照.md](docs/开发参考/开发线Git记录与工作区快照.md)\n\n"
)
if "GitHub 私人备份仓" not in text:
    text = text.replace("# MotionSLAM_ws\n\n", "# MotionSLAM_ws\n\n" + banner, 1)
    p.write_text(text, encoding="utf-8")
PY

cd "$BACKUP"
export GIT_AUTHOR_NAME="${GIT_AUTHOR_NAME:-unitree}"
export GIT_AUTHOR_EMAIL="${GIT_AUTHOR_EMAIL:-unitree@users.noreply.github.com}"
export GIT_COMMITTER_NAME="${GIT_COMMITTER_NAME:-unitree}"
export GIT_COMMITTER_EMAIL="${GIT_COMMITTER_EMAIL:-unitree@users.noreply.github.com}"

/usr/bin/git add -A
if /usr/bin/git diff --cached --quiet; then
  echo "nothing to commit — backup already matches workspace"
else
  /usr/bin/git commit -m "sync: workspace snapshot $(date +%Y-%m-%d)"
  echo "committed: $(/usr/bin/git log -1 --oneline)"
fi

echo ""
echo "To push (WiFi 临时默认路由):"
echo "  sudo ip route del default via 192.168.123.25 dev eth0"
echo "  cd $BACKUP && git push origin main"
echo "  sudo ip route add default via 192.168.123.25 dev eth0 metric 20100"
