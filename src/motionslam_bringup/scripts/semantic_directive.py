#!/usr/bin/env python3
"""SemanticDirective 校验（纯函数）."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

from task_context import TaskIdentityContext, validate_frontier_ids, validate_task_identity

SCHEMA = "motionslam.semantic_directive.v1"
ALLOWED_SEARCH_MODES = frozenset({"explore", "navigate", "hold"})


@dataclass
class DirectiveValidation:
    ok: bool
    reason: str = ""
    directive: Optional[dict[str, Any]] = None


# 向后兼容旧名
@dataclass
class DirectiveContext(TaskIdentityContext):
    """Deprecated alias; 使用 TaskIdentityContext."""

    pass


def validate_semantic_directive(
    raw: dict[str, Any],
    ctx: TaskIdentityContext,
) -> DirectiveValidation:
    schema = str(raw.get("schema", ""))
    if schema and schema != SCHEMA:
        return DirectiveValidation(False, f"REJECT_SCHEMA:{schema}")

    identity_err = validate_task_identity(
        raw,
        ctx,
        require_lifecycle=False,
        check_context_generation=True,
    )
    if identity_err:
        return DirectiveValidation(False, identity_err)

    search_mode = raw.get("search_mode")
    if search_mode is not None and str(search_mode).lower() not in ALLOWED_SEARCH_MODES:
        return DirectiveValidation(False, f"REJECT_SEARCH_MODE:{search_mode}")

    labels = raw.get("target_labels")
    if labels is not None and not isinstance(labels, list):
        return DirectiveValidation(False, "REJECT_TARGET_LABELS_TYPE")

    graph_bias = raw.get("graph_bias")
    if graph_bias is not None and not isinstance(graph_bias, dict):
        return DirectiveValidation(False, "REJECT_GRAPH_BIAS_TYPE")

    frontier_err = validate_frontier_ids(raw, ctx)
    if frontier_err:
        return DirectiveValidation(False, frontier_err)

    return DirectiveValidation(True, "ACCEPT", raw)
