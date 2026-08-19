#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from subgraph_snapshot import (  # noqa: E402
    SCHEMA,
    SubgraphBuildInput,
    build_sparse_subgraph_v1,
)


class TestSubgraphSnapshot(unittest.TestCase):
    def test_schema_and_version(self) -> None:
        snap = build_sparse_subgraph_v1(
            SubgraphBuildInput(
                version=3,
                session_id="s1",
                tile_id="main",
                map_id="m1",
                trigger_reason="periodic",
                robot_x=1.0,
                robot_y=2.0,
                robot_z=0.3,
                robot_yaw=0.5,
            )
        )
        self.assertEqual(snap["schema"], SCHEMA)
        self.assertEqual(snap["version"], 3)
        self.assertEqual(snap["session_id"], "s1")
        self.assertEqual(len(snap["nodes"]), 1)
        self.assertEqual(snap["nodes"][0]["node_id"], "robot")

    def test_keyframes_and_edges(self) -> None:
        from subgraph_snapshot import KeyframeView

        snap = build_sparse_subgraph_v1(
            SubgraphBuildInput(
                version=1,
                session_id="s",
                tile_id="t",
                map_id="m",
                trigger_reason="bt_replan",
                robot_x=0,
                robot_y=0,
                robot_z=0,
                robot_yaw=0,
                keyframes=[
                    KeyframeView(0, 0, 0, 0, 0),
                    KeyframeView(1, 1, 0, 0, 0),
                ],
            )
        )
        self.assertEqual(len(snap["nodes"]), 2)
        self.assertEqual(len(snap["edges"]), 1)
        self.assertEqual(len(snap["keyframes_meta"]), 2)


if __name__ == "__main__":
    unittest.main()
