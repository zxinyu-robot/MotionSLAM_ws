#!/usr/bin/env python3
"""semantic_bev_frame 编解码单测."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from semantic_bev_frame import (  # noqa: E402
    SemanticTokenMetaInput,
    build_semantic_token_meta,
    pack_semantic_token_frame,
    unpack_semantic_token_frame,
)


class TestSemanticBevFrame(unittest.TestCase):
    def test_roundtrip(self) -> None:
        payload = bytes([0, 50, 100] * 10)
        meta = build_semantic_token_meta(
            SemanticTokenMetaInput(
                session_id="demo",
                floor_id="f1",
                tile_id="main",
                map_id="map",
                context_generation=3,
                frame_id="world",
                grid_width=4,
                grid_height=4,
                resolution=0.15,
                origin_x=1.0,
                origin_y=2.0,
                origin_z=0.3,
                pose_x=1.1,
                pose_y=2.2,
                pose_z=0.3,
                pose_yaw_deg=45.0,
                seq=7,
                payload=payload,
            )
        )
        blob = pack_semantic_token_frame(meta, payload)
        meta2, payload2, consumed = unpack_semantic_token_frame(blob)
        self.assertEqual(consumed, len(blob))
        self.assertEqual(meta2["schema"], "motionslam.spatial_semantic_token.v1")
        self.assertEqual(meta2["seq"], 7)
        self.assertEqual(payload2, payload)


if __name__ == "__main__":
    unittest.main()
