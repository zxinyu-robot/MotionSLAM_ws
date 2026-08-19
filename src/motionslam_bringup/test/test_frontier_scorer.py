#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from frontier_scorer import rank_frontiers, select_best_frontier  # noqa: E402
from subgraph_snapshot import FrontierView  # noqa: E402


class TestFrontierScorer(unittest.TestCase):
    def setUp(self) -> None:
        self.frontiers = [
            FrontierView("kf_1", 3.0, 0.0, 0.35, geom_score=0.8),
            FrontierView("kf_2", 0.0, 4.0, 0.35, geom_score=0.6),
        ]

    def test_directive_changes_selection(self) -> None:
        directive = {
            "candidate_frontier_ids": ["kf_2"],
            "graph_bias": {"weights": {"geom": 0.2, "semantic": 0.8}},
        }
        best = select_best_frontier(self.frontiers, directive)
        assert best is not None
        self.assertEqual(best.frontier_id, "kf_2")

    def test_geom_only_without_directive(self) -> None:
        best = select_best_frontier(self.frontiers, None)
        assert best is not None
        self.assertEqual(best.frontier_id, "kf_1")

    def test_exclude_visited(self) -> None:
        ranked = rank_frontiers(self.frontiers, None, exclude_ids={"kf_1"})
        self.assertEqual(len(ranked), 1)
        self.assertEqual(ranked[0].frontier_id, "kf_2")


if __name__ == "__main__":
    unittest.main()
