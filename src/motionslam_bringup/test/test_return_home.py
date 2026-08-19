#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from return_home import (  # noqa: E402
    densify_xy,
    record_breadcrumb,
    reverse_xy_for_scan,
    should_return_home,
    uplink_is_lost,
)


class TestReturnHome(unittest.TestCase):
    def test_record_spacing(self) -> None:
        trail: list[tuple[float, float]] = []
        self.assertTrue(record_breadcrumb(trail, 0.0, 0.0, spacing_m=0.6))
        self.assertFalse(record_breadcrumb(trail, 0.2, 0.0, spacing_m=0.6))
        self.assertTrue(record_breadcrumb(trail, 0.7, 0.0, spacing_m=0.6))
        self.assertEqual(len(trail), 2)

    def test_reverse_inserts_current_when_head_far(self) -> None:
        trail = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        xy = reverse_xy_for_scan(trail, (2.9, 0.05), first_max_m=0.55)
        self.assertGreaterEqual(len(xy), 2)
        self.assertLessEqual(abs(xy[0][0] - 2.9) + abs(xy[0][1] - 0.05), 0.2)
        self.assertAlmostEqual(xy[-1][0], 0.0, places=2)

    def test_short_trail_empty(self) -> None:
        self.assertEqual(reverse_xy_for_scan([(1.0, 1.0)], (1.0, 1.0)), [])

    def test_triggers(self) -> None:
        self.assertEqual(should_return_home(explicit=True, trail_len=4), "explicit")
        self.assertEqual(should_return_home(glass_trap=True, trail_len=4), "glass_trap")
        self.assertEqual(should_return_home(link_lost=True, trail_len=4), "link_lost")
        self.assertIsNone(should_return_home(link_lost=True, trail_len=1))
        self.assertIsNone(
            should_return_home(explicit=True, already_returning=True, trail_len=8)
        )

    def test_densify_short_segment(self) -> None:
        pts = densify_xy([(0.0, 0.0), (1.0, 0.0)], step_m=0.25)
        self.assertGreaterEqual(len(pts), 5)
        self.assertAlmostEqual(pts[0][0], 0.0, places=3)
        self.assertAlmostEqual(pts[-1][0], 1.0, places=3)

    def test_uplink_lost_only_after_ok(self) -> None:
        self.assertFalse(
            uplink_is_lost(
                ever_ok=False, last_ok_mono=0.0, now_mono=30.0, timeout_s=5.0, uplink_ok=None
            )
        )
        self.assertTrue(
            uplink_is_lost(
                ever_ok=True, last_ok_mono=1.0, now_mono=10.0, timeout_s=5.0, uplink_ok=None
            )
        )
        self.assertTrue(
            uplink_is_lost(
                ever_ok=True, last_ok_mono=9.0, now_mono=10.0, timeout_s=5.0, uplink_ok=False
            )
        )


if __name__ == "__main__":
    unittest.main()
