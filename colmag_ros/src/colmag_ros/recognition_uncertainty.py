"""Package-level uncertainty gate for top-k recognition payloads."""

from __future__ import annotations

from typing import Any, Dict, List, Sequence

from colmag_ros.param_utils import finite_float


def _confidence(candidate: Dict[str, Any]) -> float:
    return max(0.0, min(1.0, finite_float(candidate.get("confidence", 0.0), 0.0)))


def apply_uncertainty_gate(candidates: Sequence[Dict[str, Any]],
                           min_confidence: float = 0.70,
                           min_margin: float = 0.20,
                           invalid_payload: bool = False) -> Dict[str, Any]:
    """Gate top-k candidates on top-1 confidence and the top-1/top-2 margin.

    Returns the acceptance decision with its reason and the values it used.
    """
    clean: List[Dict[str, Any]] = [c for c in candidates or [] if isinstance(c, dict)]
    top1 = _confidence(clean[0]) if clean else 0.0
    top2 = _confidence(clean[1]) if len(clean) > 1 else 0.0
    margin = top1 - top2 if len(clean) > 1 else top1

    reason = ""
    if invalid_payload:
        reason = "invalid_payload"
    elif not clean:
        reason = "no_candidates"
    elif top1 < float(min_confidence):
        reason = "low_confidence"
    elif margin < float(min_margin):
        reason = "low_margin"

    uncertain = bool(reason)
    return {
        "accepted": not uncertain,
        "uncertain": uncertain,
        "uncertainty_reason": reason,
        "top1_confidence": top1,
        "top2_confidence": top2,
        "margin": margin,
        "min_confidence": float(min_confidence),
        "min_margin": float(min_margin),
    }
