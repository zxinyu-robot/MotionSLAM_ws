"""semantic_mission 纯函数单测."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from geometry_msgs.msg import PoseStamped  # noqa: E402
from semantic_mission import heading_to_goal, subgoal_from_pose  # noqa: E402


def test_subgoal_from_pose_uses_body_height_when_z_zero():
    msg = PoseStamped()
    msg.header.frame_id = "world"
    msg.pose.position.x = 2.0
    msg.pose.position.y = 1.0
    msg.pose.position.z = 0.0
    sg = subgoal_from_pose(msg, body_height=0.35)
    assert sg.x == 2.0
    assert sg.y == 1.0
    assert sg.z == 0.35
    assert sg.frame_id == "world"


def test_heading_to_goal():
    qz, qw = heading_to_goal((0.0, 0.0), (1.0, 0.0))
    assert abs(qw - 1.0) < 1e-3
    qz2, qw2 = heading_to_goal((0.0, 0.0), (0.0, 1.0))
    assert abs(qz2 - 0.7071068) < 1e-3
