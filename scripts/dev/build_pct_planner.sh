#!/usr/bin/env bash
# 编译 PCT-Planner：C++ planner 库 + pct_planner_ws colcon
# 用法: ./scripts/dev/build_pct_planner.sh [--skip-thirdparty] [--skip-native]
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
PCT_SRC="$ROOT/src/PCT-Planner"
PLANNER_DIR="$PCT_SRC/pct_planner/planner"
WS="$ROOT/pct_planner_ws"

SKIP_THIRD=0
SKIP_NATIVE=0
for arg in "$@"; do
  case "$arg" in
    --skip-thirdparty) SKIP_THIRD=1 ;;
    --skip-native) SKIP_NATIVE=1 ;;
  esac
done

if [[ ! -d "$PCT_SRC" ]]; then
  echo "ERROR: 未找到 $PCT_SRC，先运行: ./scripts/dev/clone_pct_planner.sh" >&2
  exit 1
fi

if [[ "$SKIP_NATIVE" -eq 0 ]]; then
  echo "=== PCT native planner (C++/pybind) ==="
  cd "$PLANNER_DIR"
  if [[ "$SKIP_THIRD" -eq 0 ]] && [[ ! -f lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so ]]; then
    echo "→ build_thirdparty.sh (首次较慢)..."
    bash ./build_thirdparty.sh
  fi
  bash ./build.sh
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${PLANNER_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib"
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${PLANNER_DIR}/lib/build/src/common/smoothing"
  export PYTHONPATH="${PYTHONPATH:-}:${PLANNER_DIR}/lib"
fi

mkdir -p "$WS/src"
ln -sfn ../../src/PCT-Planner "$WS/src/PCT-Planner"

echo "=== colcon pct_planner (pct_planner_ws) ==="
set +u
source /opt/ros/humble/setup.bash
set -e
cd "$WS"
colcon build --symlink-install --packages-select pct_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"

echo "OK: source $WS/install/setup.bash"
