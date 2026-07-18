"""Package-level helpers for the M104 DTW template bank.

No ROS imports live here. Preprocessing and DTW distance are shared with the
runtime recognizer through ``colmag_ros.dtw_bank_features`` so offline
evaluation matches the curated runtime trajectory-DTW convention exactly.

Labels are generic strings for future recognition-label expansion, but the
current evaluated/default scope stays ``1/2/3``. Template-bank labels are
recognition labels only: they are never task commands, and no executable
mapping is created here.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from colmag_ros.dtw_bank_features import (
    dtw_distance,
    normalize_points as preprocess_points,
)


Point2D = Tuple[float, float]


def coerce_points(points: Sequence[Any]) -> List[Point2D]:
    """Keep the existing finite XY coercion used by bank files and tooling."""
    if isinstance(points, dict):
        for key in ("points", "trajectory", "path", "points_2d"):
            if key in points:
                points = points[key]
                break
    out: List[Point2D] = []
    for point in points or []:
        try:
            if isinstance(point, dict):
                x = point.get("x", point.get("u"))
                y = point.get("y", point.get("v"))
                if x is None and isinstance(point.get("position"), dict):
                    x = point["position"].get("x", point["position"].get("u"))
                    y = point["position"].get("y", point["position"].get("v"))
            else:
                x, y = point[0], point[1]
            x = float(x)
            y = float(y)
        except (TypeError, ValueError, IndexError, KeyError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            out.append((x, y))
    return out

TEMPLATE_BANK_SCHEMA = "colmag_dtw_template_bank.v1"
MODEL_TYPE = "dtw_template_bank"
DEFAULT_LABELS = ("1", "2", "3")
SEMANTIC_EXPANSION_LABELS = ("A", "C", "N", "O", "X", "L", "V")
BAD_INTENDED_LABELS = frozenset({"bad", "ambiguous", "noise", "random", "none", "na", "n/a"})

DEFAULT_PREPROCESS_PROFILE = "mouse_centroid_maxabs_resample64"
PREPROCESS_PROFILES = {
    "mouse_centroid_maxabs_resample64": {
        "resample_points": 64,
        "center": "centroid",
        "scale": "max_abs",
        "trim": "none",
    },
}

DEFAULT_BANK_DIR = Path("outputs") / "classifier_artifacts" / "dtw_bank"
DEFAULT_EVAL_DIR = Path("outputs") / "classifier_artifacts" / "dtw_eval"


def parse_labels_csv(text: Any, default: Sequence[str] = DEFAULT_LABELS) -> List[str]:
    raw = [item.strip() for item in str(text or "").split(",")]
    labels: List[str] = []
    for label in raw:
        if label and label not in labels:
            labels.append(label)
    return labels if labels else list(default)


def flip_y_points(points: Sequence[Any]) -> List[Point2D]:
    """Map canvas-normalized y-down points to the runtime y-up convention."""
    return [(x, 1.0 - y) for x, y in coerce_points(points)]


def preprocess_profile_points(points: Sequence[Any], profile: str,
                              resample_points: Optional[int] = None) -> List[Point2D]:
    params = PREPROCESS_PROFILES.get(profile)
    if params is None:
        raise ValueError("unknown preprocess profile: %r" % profile)
    n = int(resample_points or params["resample_points"])
    return [tuple(point) for point in preprocess_points(coerce_points(points), resample_length=n)]


def is_bad_intended_label(label: Any) -> bool:
    return str(label or "").strip().lower() in BAD_INTENDED_LABELS


def build_template_record(label: Any, raw_points: Sequence[Any], *,
                          source_file: str, source_type: str, y_flip: bool,
                          profile: str, index: int,
                          resample_points: Optional[int] = None,
                          source_backend: str = "capture_mouse_classifier_seed",
                          source_stroke_count: Optional[int] = None,
                          display_name: Optional[str] = None,
                          semantic_role: Optional[str] = None,
                          is_executable: bool = False,
                          notes: str = "") -> Dict[str, Any]:
    clean_label = str(label or "").strip()
    if not clean_label:
        raise ValueError("template label is required")
    points = coerce_points(raw_points)
    if len(points) < 2:
        raise ValueError("template needs at least two finite points: %s" % source_file)
    if y_flip:
        points = flip_y_points(points)
    processed = preprocess_profile_points(points, profile, resample_points)
    return {
        "id": "%s_%s_%03d" % (source_type, clean_label, index),
        "label": clean_label,
        "points": [[x, y] for x, y in processed],
        "source": {
            "source_type": str(source_type),
            "source_file": str(source_file),
            "source_backend": str(source_backend),
            "original_label": clean_label,
            "y_flip": bool(y_flip),
            "raw_point_count": len(points),
            "stroke_count": int(source_stroke_count) if source_stroke_count is not None else None,
        },
        "metadata": {
            "display_name": display_name,
            "semantic_role": semantic_role,
            "is_executable": bool(is_executable),
            "notes": str(notes or ""),
        },
    }


def build_template_bank(templates: Sequence[Dict[str, Any]], *, labels: Sequence[str],
                        profile: str, y_flip: bool,
                        resample_points: Optional[int] = None,
                        generated_by: str = "dtw_template_bank_tools",
                        created_at: Optional[float] = None) -> Dict[str, Any]:
    params = PREPROCESS_PROFILES.get(profile)
    if params is None:
        raise ValueError("unknown preprocess profile: %r" % profile)
    ordered_labels = [str(label) for label in labels]
    for template in templates:
        if str(template.get("label", "")) not in ordered_labels:
            raise ValueError("template label %r not in allowed labels %s"
                             % (template.get("label"), ordered_labels))
    return {
        "schema": TEMPLATE_BANK_SCHEMA,
        "model_type": MODEL_TYPE,
        "created_at": float(created_at if created_at is not None else time.time()),
        "generated_by": str(generated_by),
        "labels": ordered_labels,
        "allowed_labels": ordered_labels,
        "preprocess": {
            "profile": str(profile),
            "resample_points": int(resample_points or params["resample_points"]),
            "center": params["center"],
            "scale": params["scale"],
            "trim": params["trim"],
            "y_flip": bool(y_flip),
        },
        "scope": {
            "semantic_expansion": False,
            "executable_mapping": False,
            "default_validation_labels": list(DEFAULT_LABELS),
        },
        "templates": list(templates),
    }


def validate_template_bank(bank: Any) -> Dict[str, Any]:
    if not isinstance(bank, dict):
        raise ValueError("template bank must be a JSON object")
    if bank.get("schema") != TEMPLATE_BANK_SCHEMA:
        raise ValueError("unsupported template bank schema: %r" % bank.get("schema"))
    templates = bank.get("templates")
    if not isinstance(templates, list) or not templates:
        raise ValueError("template bank has no templates")
    for template in templates:
        if not isinstance(template, dict) or not str(template.get("label", "")).strip():
            raise ValueError("template bank contains a template without a label")
        points = coerce_points(template.get("points", []))
        if len(points) < 2:
            raise ValueError("template %r has fewer than two finite points"
                             % template.get("id"))
    if not isinstance(bank.get("preprocess"), dict):
        raise ValueError("template bank is missing the preprocess block")
    return bank


def load_template_bank(path: Any) -> Dict[str, Any]:
    return validate_template_bank(json.loads(Path(path).read_text(encoding="utf-8")))


def preprocess_query_for_bank(points: Sequence[Any], bank: Dict[str, Any]) -> List[Point2D]:
    """Preprocess a runtime query with the bank profile. Queries are never y-flipped."""
    preprocess = bank.get("preprocess", {})
    return preprocess_profile_points(
        points,
        str(preprocess.get("profile", DEFAULT_PREPROCESS_PROFILE)),
        preprocess.get("resample_points"),
    )


def rank_labels_for_query(query_points: Sequence[Point2D], bank: Dict[str, Any],
                          top_k: int = 3,
                          labels: Optional[Sequence[str]] = None) -> Dict[str, Any]:
    """Rank bank labels for one preprocessed query, deterministically.

    Ties break on (distance, template id); label ranking uses each label's best
    template distance. ``margin`` is the absolute distance gap between the best
    label and the nearest different label.
    """
    wanted = {str(label) for label in labels} if labels else None
    scored = []
    for template in bank.get("templates", []):
        label = str(template.get("label", ""))
        if wanted is not None and label not in wanted:
            continue
        distance = dtw_distance(query_points, coerce_points(template.get("points", [])))
        scored.append((float(distance), str(template.get("id", "")), label))
    scored.sort(key=lambda item: (item[0], item[1]))

    best_by_label: Dict[str, Tuple[float, str]] = {}
    for distance, template_id, label in scored:
        if label not in best_by_label:
            best_by_label[label] = (distance, template_id)
    ranked_labels = sorted(best_by_label.items(), key=lambda item: (item[1][0], item[0]))

    top = [
        {"rank": rank, "label": label, "distance": distance, "template_id": template_id}
        for rank, (label, (distance, template_id)) in enumerate(ranked_labels[:max(1, int(top_k))], start=1)
    ]
    best = top[0] if top else {"label": "", "distance": float("inf"), "template_id": ""}
    second = ranked_labels[1] if len(ranked_labels) > 1 else None
    second_distance = second[1][0] if second else float("inf")
    margin = second_distance - best["distance"] if second else float("inf")
    return {
        "top": top,
        "best_label": best["label"],
        "best_distance": best["distance"],
        "nearest_template": best["template_id"],
        "second_label": second[0] if second else "",
        "second_distance": second_distance,
        "margin": margin,
        "margin_ratio": margin_ratio(best["distance"], second_distance),
        "n_templates_compared": len(scored),
    }


def margin_ratio(best_distance: float, second_distance: float) -> float:
    if not (math.isfinite(best_distance) and math.isfinite(second_distance)):
        return 0.0
    if second_distance <= 1e-9:
        return 0.0
    return max(0.0, min(1.0, (second_distance - best_distance) / second_distance))


def decide_acceptance(best_distance: float, margin: float,
                      max_distance: float, min_margin: float,
                      top1_confidence: Optional[float] = None,
                      min_confidence: float = 0.0) -> Dict[str, Any]:
    reason = ""
    if not math.isfinite(best_distance):
        reason = "no_candidates"
    elif best_distance > float(max_distance):
        reason = "high_distance"
    elif math.isfinite(margin) and margin < float(min_margin):
        # margin is +inf for a single-label bank; that never fails the gate
        reason = "low_margin"
    elif top1_confidence is not None and float(top1_confidence) < float(min_confidence):
        reason = "low_confidence"
    return {
        "accepted": not reason,
        "uncertain": bool(reason),
        "uncertainty_reason": reason,
        "max_distance": float(max_distance),
        "min_margin": float(min_margin),
        "min_confidence": float(min_confidence),
    }


# --- M104-B2d opt-in runtime backend helpers (still pure / ROS-free) ---

RUNTIME_UNCERTAINTY_REASONS = {
    "no_candidates": "dtw_no_candidates",
    "high_distance": "dtw_distance_too_large",
    "low_margin": "dtw_margin_too_small",
    "low_confidence": "dtw_confidence_too_low",
}


def finite_or_none(value: Any) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def confidence_from_distance(distance: Any, max_distance: Any) -> float:
    """Monotonic distance-derived pseudo-confidence in [0, 1], not a probability."""
    try:
        parsed = float(distance)
        cap = float(max_distance)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(parsed) or cap <= 1e-9:
        return 0.0
    return max(0.0, min(1.0, 1.0 - parsed / cap))


def load_template_bank_state(path: Any,
                             allowed_labels: Sequence[str] = DEFAULT_LABELS) -> Dict[str, Any]:
    """Load and label-filter a bank for the opt-in runtime backend; never raises.

    ``status`` is ``ready`` / ``missing_path`` / ``load_error`` / ``unavailable``;
    ``reason`` is the runtime uncertainty reason a non-ready state publishes.
    Templates outside ``allowed_labels`` (e.g. future semantic labels) are
    ignored unless explicitly allowed.
    """
    labels = [str(label) for label in allowed_labels] or list(DEFAULT_LABELS)
    state: Dict[str, Any] = {
        "status": "missing_path",
        "reason": "dtw_template_bank_missing_path",
        "detail": "dtw_template_bank_path is empty",
        "path": str(path or "").strip(),
        "bank": None,
        "bank_name": "",
        "labels": labels,
        "template_count": 0,
        "ignored_template_count": 0,
    }
    if not state["path"]:
        return state
    try:
        bank = load_template_bank(state["path"])
    except Exception as exc:
        state.update(status="load_error", reason="dtw_template_bank_unavailable",
                     detail="failed to load template bank: %s" % exc)
        return state
    wanted = set(labels)
    templates = [t for t in bank["templates"] if str(t.get("label", "")) in wanted]
    ignored = len(bank["templates"]) - len(templates)
    if not templates:
        state.update(status="unavailable", reason="dtw_template_bank_unavailable",
                     detail="no templates match allowed labels %s" % ",".join(labels),
                     ignored_template_count=ignored)
        return state
    state.update(
        status="ready", reason="", detail="",
        bank=dict(bank, templates=templates),
        bank_name=Path(state["path"]).stem,
        template_count=len(templates),
        ignored_template_count=ignored,
    )
    return state


def bank_candidates_from_ranking(ranking: Dict[str, Any], max_distance: Any,
                                 backend: str = "dtw_template_bank",
                                 template_bank_name: str = "") -> List[Dict[str, Any]]:
    """Convert one label ranking into ``/colmag/symbol_candidates`` candidates.

    Rank 1 carries the ranking margin (best vs nearest different label); lower
    ranks carry their distance gap behind the best label.
    """
    best_distance = float(ranking.get("best_distance", float("inf")))
    candidates = []
    for item in ranking.get("top", []):
        distance = float(item["distance"])
        if not math.isfinite(distance):
            continue
        margin = ranking.get("margin") if int(item["rank"]) == 1 else distance - best_distance
        candidates.append({
            "rank": int(item["rank"]),
            "label": str(item["label"]),
            "confidence": confidence_from_distance(distance, max_distance),
            "distance": distance,
            "margin": finite_or_none(margin),
            "backend": str(backend),
            "template_bank_name": str(template_bank_name),
            "template_id": str(item.get("template_id", "")),
        })
    return candidates


def _runtime_gate(accepted: bool, reason: str, candidates: Sequence[Dict[str, Any]],
                  max_distance: Any, min_margin: Any, min_confidence: Any,
                  best_distance: Any = None, margin: Any = None) -> Dict[str, Any]:
    return {
        "accepted": bool(accepted),
        "uncertain": not accepted,
        "uncertainty_reason": "" if accepted else str(reason),
        "top1_confidence": float(candidates[0]["confidence"]) if candidates else 0.0,
        "top2_confidence": float(candidates[1]["confidence"]) if len(candidates) > 1 else 0.0,
        "best_distance": finite_or_none(best_distance),
        "margin": finite_or_none(margin),
        "max_distance": float(max_distance),
        "min_margin": float(min_margin),
        "min_confidence": float(min_confidence),
    }


def recognize_with_template_bank(points: Sequence[Any], state: Optional[Dict[str, Any]],
                                 top_k: int = 3, max_distance: float = 0.12,
                                 min_margin: float = 0.01,
                                 min_confidence: float = 0.30):
    """Pure opt-in runtime path: capture points -> (candidates, gate, metadata).

    Runtime query points are never y-flipped; the y-flip decision lives in the
    bank (seed import side). A non-ready state yields an uncertain gate with no
    candidates instead of crashing or silently falling back to another backend.
    """
    state = state or {"status": "unavailable", "reason": "dtw_template_bank_unavailable"}
    metadata: Dict[str, Any] = {
        "dtw_template_bank_status": str(state.get("status", "unavailable")),
        "template_bank_path": str(state.get("path", "")),
        "template_bank_name": str(state.get("bank_name", "")),
        "template_count": int(state.get("template_count", 0) or 0),
        "template_bank_labels": [str(label) for label in state.get("labels", [])],
    }
    if state.get("detail"):
        metadata["dtw_template_bank_detail"] = str(state["detail"])
    if state.get("status") != "ready":
        reason = str(state.get("reason") or "dtw_template_bank_unavailable")
        return [], _runtime_gate(False, reason, [], max_distance, min_margin,
                                 min_confidence), metadata

    bank = state["bank"]
    query = preprocess_query_for_bank(points, bank)
    ranking = rank_labels_for_query(query, bank, top_k=top_k, labels=state.get("labels"))
    candidates = bank_candidates_from_ranking(
        ranking, max_distance, template_bank_name=str(state.get("bank_name", "")))
    if not candidates:
        return [], _runtime_gate(False, "dtw_no_candidates", [], max_distance,
                                 min_margin, min_confidence), metadata

    decision = decide_acceptance(ranking["best_distance"], ranking["margin"],
                                 max_distance, min_margin,
                                 top1_confidence=candidates[0]["confidence"],
                                 min_confidence=min_confidence)
    reason = RUNTIME_UNCERTAINTY_REASONS.get(decision["uncertainty_reason"],
                                             decision["uncertainty_reason"])
    metadata.update({
        "nearest_template": ranking["nearest_template"],
        "second_label": ranking["second_label"],
        "margin_ratio": ranking["margin_ratio"],
        "n_templates_compared": ranking["n_templates_compared"],
    })
    gate = _runtime_gate(decision["accepted"], reason, candidates, max_distance,
                         min_margin, min_confidence,
                         best_distance=ranking["best_distance"], margin=ranking["margin"])
    return candidates, gate, metadata
