#!/usr/bin/env python3
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from plan_fail_recovery import (  # noqa: E402
    DriftThresholds,
    RecoveryBranch,
    assess_drift,
    parse_drift_estimate,
    should_pause_replan,
    yaw_delta,
)


class TestPlanFailRecovery(unittest.TestCase):
    def test_parse_drift_estimate(self) -> None:
        trans, yaw = parse_drift_estimate('{"trans_m": 0.31, "yaw_deg": 5.2}')
        self.assertAlmostEqual(trans, 0.31)
        self.assertAlmostEqual(yaw, 5.2)

    def test_pgo_drift_triggers_icp(self) -> None:
        result = assess_drift(
            drift_estimate_raw='{"trans_m": 0.40, "yaw_deg": 2.0}',
            thresholds=DriftThresholds(trans_m=0.25, yaw_deg=8.0),
        )
        self.assertTrue(result.drifted)
        self.assertEqual(result.branch, RecoveryBranch.DRIFT_ICP)
        self.assertEqual(result.reason, "pgo_drift_estimate")

    def test_yaw_drift_triggers_icp(self) -> None:
        result = assess_drift(
            drift_estimate_raw='{"trans_m": 0.05, "yaw_deg": 12.0}',
        )
        self.assertTrue(result.drifted)
        self.assertEqual(result.reason, "pgo_drift_estimate")

    def test_odom_jump_triggers_icp(self) -> None:
        result = assess_drift(
            odom_jump_m=0.55,
            thresholds=DriftThresholds(odom_jump_m=0.45),
        )
        self.assertTrue(result.drifted)
        self.assertEqual(result.reason, "odom_jump")

    def test_no_drift_goes_escape(self) -> None:
        result = assess_drift(
            drift_estimate_raw='{"trans_m": 0.05, "yaw_deg": 1.0}',
            odom_jump_m=0.1,
            anchor_trans_m=0.1,
            anchor_yaw_deg=2.0,
        )
        self.assertFalse(result.drifted)
        self.assertEqual(result.branch, RecoveryBranch.ESCAPE)

    def test_should_pause_replan(self) -> None:
        self.assertFalse(should_pause_replan(replan_fail_count=1, pause_after=2))
        self.assertTrue(should_pause_replan(replan_fail_count=2, pause_after=2))
        self.assertTrue(
            should_pause_replan(replan_fail_count=1, pause_after=2, emergency_bspline=True)
        )

    def test_yaw_delta(self) -> None:
        self.assertAlmostEqual(yaw_delta(0.0, math.pi / 2), math.pi / 2, places=5)


if __name__ == "__main__":
    unittest.main()
