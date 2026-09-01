from __future__ import annotations

from typing import Iterable, Optional

from .types import GraspResult


def selection_score(result: GraspResult) -> float:
    """Deterministic quality ranking; this is not a grasp-success probability."""
    if not result.valid:
        return float("-inf")
    depth_support = float(result.diagnostics.get("rim_depth_support", 0.0))
    edge_support = float(result.diagnostics.get("edge_support", 0.0))
    return (
        0.55 * float(result.geometry_score)
        + 0.30 * float(result.detection_score)
        + 0.10 * depth_support
        + 0.05 * edge_support
    )


def select_best_result(results: Iterable[GraspResult]) -> Optional[GraspResult]:
    items = list(results)
    for item in items:
        item.diagnostics["selected"] = False
        item.diagnostics["selection_score"] = None
    valid = [item for item in items if item.valid]
    if not valid:
        return None
    for item in valid:
        item.diagnostics["selection_score"] = selection_score(item)
    # Object ID is the final deterministic tie-breaker; no random arm motion target.
    best = max(valid, key=lambda item: (selection_score(item), item.geometry_score,
                                        item.detection_score, item.object_id))
    best.diagnostics["selected"] = True
    return best
