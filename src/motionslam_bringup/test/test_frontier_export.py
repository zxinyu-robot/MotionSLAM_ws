#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from frontier_export import FrontierExportConfig, export_frontiers_from_keyframes  # noqa: E402
from subgraph_snapshot import KeyframeView  # noqa: E402


class TestFrontierExport(unittest.TestCase):
    def test_keyframe_and_ray_frontiers(self) -> None:
        kfs = [
            KeyframeView(0, 0.0, 0.0, 0.0, 0.0),
            KeyframeView(1, 3.0, 0.0, 0.0, 0.0),
            KeyframeView(2, 0.0, 4.0, 0.0, 0.0),
        ]
        frontiers = export_frontiers_from_keyframes(
            0.0,
            0.0,
            0.35,
            0.0,
            kfs,
            config=FrontierExportConfig(min_dist_m=1.5, min_ray_frontiers=0),
        )
        ids = {f.frontier_id for f in frontiers}
        self.assertIn("kf_1", ids)
        self.assertIn("kf_2", ids)
        self.assertNotIn("kf_0", ids)

    def test_ray_fallback_when_sparse(self) -> None:
        frontiers = export_frontiers_from_keyframes(
            0.0,
            0.0,
            0.35,
            0.0,
            [],
            config=FrontierExportConfig(min_ray_frontiers=4, max_frontiers=8),
        )
        self.assertGreaterEqual(len(frontiers), 4)
        self.assertTrue(any(f.frontier_id.startswith("ray_") for f in frontiers))


if __name__ == "__main__":
    unittest.main()
