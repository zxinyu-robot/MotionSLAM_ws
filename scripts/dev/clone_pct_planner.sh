#!/usr/bin/env bash
# 拉取 PCT-Planner（VectorRobotics ROS2 fork）到 src/PCT-Planner
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
TARGET="$ROOT/src/PCT-Planner"
URL="${PCT_PLANNER_GIT_URL:-https://github.com/VectorRobotics/PCT_planner.git}"
BRANCH="${PCT_PLANNER_GIT_BRANCH:-main}"

if [[ -d "$TARGET/.git" ]]; then
  echo "PCT-Planner 已存在: $TARGET"
  git -C "$TARGET" fetch --depth 1 origin "$BRANCH" 2>/dev/null || true
  git -C "$TARGET" checkout "$BRANCH" 2>/dev/null || true
  git -C "$TARGET" pull --ff-only origin "$BRANCH" 2>/dev/null || true
  exit 0
fi

echo "Cloning PCT-Planner → $TARGET"
if ! git clone --depth 1 --branch "$BRANCH" "$URL" "$TARGET" 2>/dev/null; then
  echo "直连失败，尝试 ghfast 镜像..."
  git clone --depth 1 --branch "$BRANCH" \
    "https://ghfast.top/${URL}" "$TARGET"
fi

mkdir -p "$ROOT/pct_planner_ws/src"
ln -sfn ../../src/PCT-Planner "$ROOT/pct_planner_ws/src/PCT-Planner"
echo "OK: $TARGET ($(git -C "$TARGET" log -1 --oneline))"
