#!/usr/bin/env python3
"""验收脚本统一入口 — 替代多个 verify_*.py 直调.

用法:
  python3 scripts/verify.py f2 --static --duration 15
  python3 scripts/verify.py f3 --once
  python3 scripts/verify.py f4 --once
  python3 scripts/verify.py h5 --distance 2
  python3 scripts/verify.py h5-nav2 --distance 2    # hybrid / Nav2
  python3 scripts/verify.py calibrate --once --report-only
"""
from __future__ import annotations

import argparse
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

MODULES = {
    "f2": "verify_f2_drift.py",
    "f3": "verify_f3_demo_cloud.py",
    "f4": "verify_f4_demo_scan.py",
    "h5": "verify_scan_planner_h5.py",
    "h5-nav2": "verify_h5_arrival.py",
    "calibrate": "calibrate_odom_robo.py",
    "ate-static": "verify_ate_static_drift.py",
    "obstacle": "check_obstacle_avoid.py",
}


def main() -> int:
    parser = argparse.ArgumentParser(description="MotionSLAM 验收 CLI")
    parser.add_argument(
        "command",
        choices=sorted(MODULES.keys()),
        help="验收子命令",
    )
    args, rest = parser.parse_known_args()
    target = ROOT / MODULES[args.command]
    if not target.is_file():
        print(f"ERROR: 缺少 {target}", file=sys.stderr)
        return 1
    sys.argv = [str(target), *rest]
    runpy.run_path(str(target), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
