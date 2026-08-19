#!/usr/bin/env python3
"""边侧离线 SC-PGO + 可选 PCD merge → map_reloc.pcd（骨架，待接真机 session 数据）."""
from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline SC-PGO session processor")
    parser.add_argument("--session", required=True, help="export_sc_pgo_session.sh 输出目录")
    parser.add_argument(
        "--output",
        default="",
        help="map_reloc.pcd 输出路径（默认 <session>/maps/map_reloc.pcd）",
    )
    args = parser.parse_args()

    session = os.path.abspath(args.session)
    manifest = os.path.join(session, "session_manifest.yaml")
    if not os.path.isdir(session):
        print(f"ERROR: session 目录不存在: {session}", file=sys.stderr)
        return 1
    if not os.path.isfile(manifest):
        print(f"WARN: 无 manifest: {manifest}", file=sys.stderr)

    out = args.output or os.path.join(session, "maps", "map_reloc.pcd")
    octovox = os.path.join(session, "maps", "OctVoxMap.pcd")
    if os.path.isfile(octovox):
        os.makedirs(os.path.dirname(out), exist_ok=True)
        # 骨架：直接复制 OctVox 作为占位；完整 PGO+ICP 在工控机扩展
        import shutil

        shutil.copy2(octovox, out)
        print(f"OK (skeleton): {out} ← OctVoxMap.pcd")
        print("TODO: 全量 SC-PGO + ICP warp/merge 在此脚本扩展")
        return 0

    print("ERROR: 无 OctVoxMap.pcd，请先 Super-LIO save_map 或手动放入 maps/", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
