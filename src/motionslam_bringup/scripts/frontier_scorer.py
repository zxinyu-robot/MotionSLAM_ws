#!/usr/bin/env python3
"""FrontierScorer：几何分 + SemanticDirective graph_bias（P2-2）."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from subgraph_snapshot import FrontierView


DEFAULT_WEIGHTS = {"geom": 0.5, "semantic": 0.5}


@dataclass
class FrontierScore:
    frontier_id: str
    x: float
    y: float
    z: float
    geom_score: float
    semantic_score: float
    total_score: float


def _frontier_node_id(frontier_id: str) -> str:
    if frontier_id.startswith("kf_"):
        return f"node_{frontier_id}"
    return frontier_id


def graph_bias(frontier: FrontierView, directive: Optional[dict[str, Any]]) -> float:
    if not directive:
        return 0.0
    score = 0.0
    fid = str(frontier.frontier_id)
    candidates = {str(x) for x in directive.get("candidate_frontier_ids") or []}
    if fid in candidates:
        score += 1.0

    bias = directive.get("graph_bias") or {}
    preferred = {str(x) for x in bias.get("preferred_node_ids") or []}
    avoid = {str(x) for x in bias.get("avoid_node_ids") or []}
    node_id = _frontier_node_id(fid)
    if node_id in preferred or fid in preferred:
        score += 0.5
    if node_id in avoid or fid in avoid:
        score -= 1.0
    return score


def score_frontier(
    frontier: FrontierView,
    directive: Optional[dict[str, Any]],
    *,
    weights: Optional[dict[str, float]] = None,
) -> FrontierScore:
    w = dict(DEFAULT_WEIGHTS)
    if directive:
        bias = directive.get("graph_bias") or {}
        raw_w = bias.get("weights") or {}
        if isinstance(raw_w, dict):
            for key in ("geom", "semantic"):
                if key in raw_w:
                    try:
                        w[key] = float(raw_w[key])
                    except (TypeError, ValueError):
                        pass
    if weights:
        w.update(weights)

    geom = max(0.0, min(1.0, float(frontier.geom_score)))
    sem = graph_bias(frontier, directive)
    sem_norm = max(0.0, min(1.0, (sem + 1.0) / 2.0))
    total = w["geom"] * geom + w["semantic"] * sem_norm
    return FrontierScore(
        frontier_id=frontier.frontier_id,
        x=frontier.x,
        y=frontier.y,
        z=frontier.z,
        geom_score=geom,
        semantic_score=sem,
        total_score=total,
    )


def rank_frontiers(
    frontiers: list[FrontierView],
    directive: Optional[dict[str, Any]],
    *,
    exclude_ids: Optional[set[str]] = None,
) -> list[FrontierScore]:
    blocked = exclude_ids or set()
    scored = [
        score_frontier(f, directive)
        for f in frontiers
        if f.frontier_id not in blocked
    ]
    scored.sort(key=lambda s: s.total_score, reverse=True)
    return scored


def select_best_frontier(
    frontiers: list[FrontierView],
    directive: Optional[dict[str, Any]],
    *,
    exclude_ids: Optional[set[str]] = None,
) -> Optional[FrontierScore]:
    ranked = rank_frontiers(frontiers, directive, exclude_ids=exclude_ids)
    return ranked[0] if ranked else None
