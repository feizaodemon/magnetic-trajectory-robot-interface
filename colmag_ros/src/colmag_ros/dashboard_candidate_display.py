"""ROS-free dashboard candidate display formatters.

This module owns text-only candidate/rank/recognizer display formatting. It
does not publish, confirm, dispatch tasks, import Tk, or touch Gazebo / robot
routes.
"""

from dataclasses import dataclass

from colmag_ros.dashboard_dwell_status import format_trajectory_preview_interaction_text


def backend_display_name(backend, recognition_model_path=""):
    return {
        "dtw": "DTW",
        "dtw_template_bank": "DTW template bank",
        "lightweight": "Lightweight (DTW)",
        "fake": "Fake (offline)",
    }.get(str(backend or "").strip().lower(), "Unknown")


def candidate_backend_status_text(payload):
    """Operator-facing backend line for /colmag/symbol_candidates payloads."""
    if not isinstance(payload, dict):
        return "Backend: unknown"

    backend = str(payload.get("backend") or payload.get("active_backend") or "unknown")
    fallback = payload.get("fallback_active", False)
    fallback_reason = str(payload.get("fallback_reason", "none"))

    parts = ["Backend: %s" % backend]
    feature = str(payload.get("feature_mode") or "").strip()
    if fallback:
        parts.append("Fallback: %s" % fallback_reason)
    if feature:
        parts.append("Feature: %s" % feature)
    return " | ".join(parts)


def candidate_display_name(candidate):
    """Display-only label text for a candidate row; never affects dispatch."""
    try:
        from colmag_ros.m104c2c3_display_semantics import display_label_for_candidate
    except Exception:
        display_label_for_candidate = None
    if display_label_for_candidate is not None:
        try:
            label = display_label_for_candidate(candidate)
            if label:
                return label
        except Exception:
            pass
    if isinstance(candidate, dict):
        return str(candidate.get("display_name", "") or "")
    return ""


def summarize_candidates(payload):
    if not payload:
        return []
    candidates = payload.get("candidates", [])
    if not isinstance(candidates, list):
        return []

    out = []
    for candidate in candidates:
        if isinstance(candidate, dict) and "rank" in candidate and "label" in candidate:
            try:
                rank = int(candidate["rank"])
                label = str(candidate["label"])
                confidence = float(candidate.get("confidence", 0.0))
                out.append((rank, label, confidence))
            except (TypeError, ValueError):
                pass
    out.sort(key=lambda x: x[0])
    return out[:3]


def _candidate_rows(payload, *, include_rejected=False):
    if not isinstance(payload, dict):
        return []
    candidates = payload.get("candidates", [])
    if include_rejected and not candidates:
        candidates = payload.get("rejected_candidates", [])
    if not isinstance(candidates, list):
        return []

    out = []
    for candidate in candidates:
        if isinstance(candidate, dict) and "rank" in candidate and "label" in candidate:
            try:
                out.append({
                    "rank": int(candidate["rank"]),
                    "label": str(candidate["label"]),
                    "confidence": float(candidate.get("confidence", 0.0)),
                    "candidate": candidate,
                })
            except (TypeError, ValueError):
                pass
    out.sort(key=lambda item: item["rank"])
    return out


def format_rank_rows(payload, count=3):
    """Return dashboard rank-card row text for a candidate payload."""
    tops = summarize_candidates(payload)
    candidates = payload.get("candidates", []) if isinstance(payload, dict) else []
    candidate_by_label = {
        str(candidate.get("label", "")): candidate
        for candidate in candidates
        if isinstance(candidate, dict)
    }
    rows = []
    for index in range(int(count)):
        if index < len(tops):
            rank, label, confidence = tops[index]
            display = candidate_display_name(candidate_by_label.get(label, {}))
            if display:
                rows.append("Rank %d: %s | %s | %.3f" % (rank, label, display, confidence))
            else:
                rows.append("Rank %d: %s | %.3f" % (rank, label, confidence))
        else:
            rows.append("Rank %d: -" % (index + 1))
    return rows


def format_candidate_rows_for_display(payload, count=3):
    """Rows for operator display, including rejected top-k for uncertain DTW."""
    rows = []
    candidates = _candidate_rows(payload, include_rejected=True)
    rejected = isinstance(payload, dict) and not payload.get("candidates") and bool(payload.get("rejected_candidates"))
    for index in range(int(count)):
        if index < len(candidates):
            item = candidates[index]
            display = candidate_display_name(item["candidate"])
            suffix = " rejected" if rejected else ""
            if display:
                rows.append("Rank %d%s: %s | %s | %.3f" % (
                    item["rank"], suffix, item["label"], display, item["confidence"]))
            else:
                rows.append("Rank %d%s: %s | %.3f" % (
                    item["rank"], suffix, item["label"], item["confidence"]))
        else:
            rows.append("Rank %d: -" % (index + 1))
    return rows


def _format_optional_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "-"
    return "%.3f" % number


def format_candidate_result_summary(payload):
    """Compact accepted/rejected/uncertain line for DTW dashboard debugging."""
    if not isinstance(payload, dict):
        return "Result: waiting for candidates"
    rows = _candidate_rows(payload, include_rejected=True)
    top = rows[0] if rows else None
    if payload.get("accepted") is True:
        result = "accepted"
    elif payload.get("uncertain") or payload.get("accepted") is False:
        result = "uncertain"
    else:
        result = "received"
    reason = str(payload.get("uncertainty_reason") or payload.get("reason") or "").strip()
    parts = ["Result: %s" % result]
    if top is not None:
        parts.append("top-1 %s %.3f" % (top["label"], top["confidence"]))
    else:
        parts.append("top-1 -")
    if reason:
        parts.append("reason %s" % reason)
    if any(key in payload for key in ("best_distance", "margin")):
        parts.append("best_distance %s" % _format_optional_float(payload.get("best_distance")))
        parts.append("margin %s" % _format_optional_float(payload.get("margin")))
    return " | ".join(parts)


def format_operator_recognition_summary(payload):
    """Recognition result without backend/debug-reason implementation details."""
    if not isinstance(payload, dict):
        return "No candidates yet"
    rows = _candidate_rows(payload, include_rejected=True)
    if not rows:
        return "No candidates yet"
    accepted = payload.get("accepted") is True
    state = "Accepted" if accepted else "Not accepted"
    return "Recognized symbol: %s | Confidence: %.0f%% | %s" % (
        rows[0]["label"], rows[0]["confidence"] * 100.0, state)


def format_mapped_action(action_id):
    """Humanize an existing action identifier without changing its payload."""
    tokens = [token for token in str(action_id or "").strip().split("_") if token]
    if not tokens:
        return ""
    abbreviations = {"DTW", "FCI", "FR3", "OCR", "ROS", "USB"}
    words = [token if token.upper() in abbreviations else token.lower()
             for token in tokens]
    words[0] = words[0] if words[0] in abbreviations else words[0].capitalize()
    return " ".join(words)


def format_operator_recognition_view(payload, status="", sample_ready=False):
    """Return user-facing recognition fields without DTW implementation details."""
    rows = _candidate_rows(payload, include_rejected=True)
    if not rows:
        if status in ("recognizing", "symbol_capture_published", "RECOGNIZING"):
            headline = "Recognition in progress…"
        elif sample_ready:
            headline = "Trajectory ready for recognition"
        else:
            headline = "No candidates yet"
        return {"headline": headline, "mapping": "", "rows": ["", "", ""]}

    top = rows[0]
    accepted = isinstance(payload, dict) and payload.get("accepted") is True
    result = "Accepted" if accepted else "Not accepted"
    mapping_id = str(top["candidate"].get("task") or "").strip()
    meaningful = [item for item in rows[1:3] if item["confidence"] > 0.0005]
    zero_confidence = [item for item in rows[1:3] if item["confidence"] <= 0.0005]
    secondary = [
        "Rank %d: %s · confidence %.0f%%" % (
            item["rank"], item["label"], item["confidence"] * 100.0)
        for item in meaningful
    ]
    if zero_confidence:
        secondary.append("Other candidates: %s" % ", ".join(
            item["label"] for item in zero_confidence))
    return {
        "headline": "Recognized symbol: %s" % top["label"],
        "mapping": "Mapped action: %s" % format_mapped_action(mapping_id)
        if mapping_id else "",
        "rows": (["Confidence: %.0f%% · %s" % (
            top["confidence"] * 100.0, result)] + secondary + ["", ""])[:3],
    }


def format_candidate_debug_details(payload):
    """Expose DTW/template evidence only in the opt-in diagnostics panel."""
    rows = _candidate_rows(payload, include_rejected=True)
    top = rows[0]["candidate"] if rows else {}
    details = []
    if isinstance(payload, dict) and "best_distance" in payload:
        details.append("DTW distance: %s" % _format_optional_float(
            payload.get("best_distance")))
    template_name = str(top.get("template_bank_name") or "").strip()
    template_id = str(top.get("template_id") or "").strip()
    if template_name:
        details.append("Template bank: %s" % template_name)
    if template_id:
        details.append("Template: %s" % template_id)
    action_id = str(top.get("task") or "").strip()
    if action_id:
        details.append("Action ID: %s" % action_id)
    return " | ".join(details) or "DTW/template details: unavailable"


def derive_operator_workflow_stage(
    interaction_state="",
    has_sample=False,
    has_candidate=False,
):
    """Derive a presentation stage from existing sample/candidate/action state."""
    interaction = str(interaction_state or "").strip()
    if interaction in ("preview_confirmed", "rejected", "cleared", "BLOCKED"):
        return "act"
    if has_candidate or interaction == "CONFIRM_PENDING":
        return "review"
    if interaction in ("recognizing", "symbol_capture_published", "RECOGNIZING"):
        return "recognize"
    if has_sample:
        return "recognize"
    return "draw"


def format_operator_workflow(stage, interaction_profile="real_board"):
    if interaction_profile == "mouse":
        steps = (("start", "Start drawing"), ("draw", "Draw"),
                 ("recognize", "Auto-recognize"), ("review", "Review"),
                 ("act", "Act"))
    else:
        steps = (("draw", "Draw"), ("recognize", "Recognize"),
                 ("review", "Review"), ("act", "Act"))
    current_index = next(
        (index for index, (key, _label) in enumerate(steps) if key == stage), 0)
    units = []
    for index, (_key, label) in enumerate(steps):
        label = label.replace(" ", "\u00a0")
        if index < current_index:
            units.append("✓\u00a0%s" % label)
        elif index == current_index:
            units.append("●\u00a0%s" % label)
        else:
            units.append(label)
    return "%s\n%s" % (
        "  →  ".join(units[:2]), "  →  ".join(units[2:]))


def format_operator_candidate_rows(candidate_rows, status="", sample_ready=False):
    rows = [str(row or "") for row in candidate_rows]
    visible = [row for row in rows if row and not row.rstrip().endswith("-")]
    if visible:
        return (visible + [""] * 3)[:3]
    if status in ("recognizing", "symbol_capture_published", "RECOGNIZING"):
        message = "Recognition in progress…"
    elif sample_ready:
        message = "Hover B Recognize to submit the trajectory."
    else:
        message = "Draw a character in the center."
    return [message, "", ""]


def format_recognition_labels(labels, max_visible=8):
    if not isinstance(labels, (list, tuple)) or not labels:
        return "Labels: unknown"
    cleaned = [str(label) for label in labels if str(label) != ""]
    if not cleaned:
        return "Labels: unknown"
    limit = max(1, int(max_visible))
    shown = cleaned[:limit]
    text = "Labels: %s" % ", ".join(shown)
    remaining = len(cleaned) - len(shown)
    if remaining > 0:
        text = "%s ... +%d" % (text, remaining)
    return text


def format_preview_candidate_rows(candidates, count=3):
    """Return legacy right-panel preview candidate rows."""
    rows = []
    for index in range(int(count)):
        if index < len(candidates):
            candidate = candidates[index]
            rows.append("Candidate %d: %s  score=%.2f" % (
                index + 1,
                candidate.get("label", "-"),
                float(candidate.get("confidence", 0.0)),
            ))
        else:
            rows.append("Candidate %d: -" % (index + 1))
    return rows


@dataclass(frozen=True)
class PreviewCandidateDisplayState:
    status: str
    point_count: int
    backend: str
    raw_point_count: object = None
    frozen_point_count: object = None
    published_point_count: object = None
    sample_lifecycle_phase: object = None
    published_sample_raw_point_count: object = None
    board_sample_cleanup_enabled: bool = True
    result_payload: object = None
    external_candidate_payload: object = None
    recognizer_detail: str = ""
    hover_button: str = ""
    hover_source: str = "none"
    dwell_progress: float = 0.0
    hover_progress_enabled: bool = True
    preview_interaction_state: object = None
    preview_confirmed_label: str = ""
    trajectory_candidates: object = None
    sample_ready: bool = False


def format_operator_action_status(
    interaction_state="",
    confirmed_label="",
    top_candidate_label="",
    sample_ready=False,
):
    """Format operator feedback from existing interaction/candidate state."""
    interaction = str(interaction_state or "").strip()
    if interaction == "rejected":
        return "Candidate rejected"
    if interaction == "cleared":
        return "Drawing cleared"
    if interaction in ("recognizing", "symbol_capture_published"):
        return "Wait for recognition"
    if interaction == "preview_confirmed" and confirmed_label:
        return "Confirmed: %s" % confirmed_label
    if top_candidate_label:
        return "Review the result, then choose C / A / X"
    if sample_ready:
        return "Hover B Recognize"
    return "Draw a character in the center"


def operator_action_status_text(state):
    """Derive concise operator feedback from the existing preview state."""
    if not isinstance(state, PreviewCandidateDisplayState):
        raise TypeError("state must be PreviewCandidateDisplayState")
    if state.external_candidate_payload:
        tops = summarize_candidates(state.external_candidate_payload)
    else:
        tops = [
            (int(candidate.get("rank", index + 1)), str(candidate.get("label", "")),
             float(candidate.get("confidence", 0.0)))
            for index, candidate in enumerate(state.trajectory_candidates or [])
            if isinstance(candidate, dict) and candidate.get("label")
        ]
    return format_operator_action_status(
        interaction_state=state.preview_interaction_state or state.status,
        confirmed_label=state.preview_confirmed_label,
        top_candidate_label=tops[0][1] if tops else "",
        sample_ready=state.sample_ready,
    )


def _format_preview_status_and_points(state):
    label_map = {
        "idle": "idle",
        "collecting": "collecting (draw more)",
        "recognizing": "recognizing",
        "ready": "candidates ready",
        "none": "no candidate",
        "no_text": "no OCR text",
        "unavailable": "recognizer unavailable",
        "error": "recognizer error",
        "rejected": "rejected",
        "uncertain": "uncertain / redraw",
        "symbol_capture_published": "sample sent to recognizer",
        "too_few_drawing_points": "too few drawing points",
    }
    if state.sample_lifecycle_phase is not None:
        label_map["no_text"] = "no candidate"
    status_text = "Candidate status: %s" % label_map.get(state.status, state.status)
    counts = (state.raw_point_count, state.frozen_point_count, state.published_point_count)
    if all(count is None for count in counts):
        return status_text, "Points collected: %d" % state.point_count, None, None

    raw_points = int(state.raw_point_count or 0)
    frozen_points = int(state.frozen_point_count or 0)
    published_points = int(state.published_point_count or 0)
    points_text = "Live points: %d | Raw drawing: %d | Frozen raw: %d | Published sample: %d" % (
        state.point_count, raw_points, frozen_points, published_points)
    sample_text = "Sample: %s | Live %d | Raw %d | Frozen raw %d | Published %d" % (
        state.sample_lifecycle_phase or "idle",
        state.point_count,
        raw_points,
        frozen_points,
        published_points,
    )
    cleanup_text = "Published sample: raw %d -> clean %d | cleanup %s | controls excluded" % (
        int(state.published_sample_raw_point_count or 0),
        published_points,
        "on" if state.board_sample_cleanup_enabled else "off",
    )
    return status_text, points_text, sample_text, cleanup_text


def _format_preview_backend(state):
    backend_txt = "Backend: %s" % state.backend
    detail_txt = "Feature: trajectory_dtw | Source: /colmag/symbol_candidates"
    return backend_txt, detail_txt


def preview_candidate_ui_texts(state):
    """Build pure text lines for the dashboard Recognition side panel."""
    if not isinstance(state, PreviewCandidateDisplayState):
        raise TypeError("state must be PreviewCandidateDisplayState")

    status_txt, points_txt, sample_txt, cleanup_txt = _format_preview_status_and_points(state)
    backend_txt, recognizer_detail_txt = _format_preview_backend(state)

    progress_map = {
        "idle": "idle",
        "collecting": "collecting",
        "recognizing": "recognizing",
        "ready": "top candidates ready",
        "no_text": "no OCR text",
        "none": "no candidate",
        "unavailable": "unavailable",
        "error": "error",
        "rejected": "rejected",
        "uncertain": "uncertain",
        "symbol_capture_published": "sample sent",
    }
    progress_txt = "Progress: %s" % progress_map.get(state.status, state.status)
    interaction_txt = format_trajectory_preview_interaction_text(
        state.preview_interaction_state if state.preview_interaction_state is not None else state.status,
        state.hover_source,
        state.hover_button,
        state.dwell_progress,
        hover_progress_enabled=state.hover_progress_enabled,
    )
    confirm_txt = "Confirm: disabled in preview"
    if state.preview_confirmed_label:
        confirm_txt = "Preview confirm: %s" % state.preview_confirmed_label

    if state.external_candidate_payload:
        candidate_rows = format_candidate_rows_for_display(state.external_candidate_payload, count=3)
    else:
        candidate_rows = format_preview_candidate_rows(state.trajectory_candidates or [])

    return {
        "status": status_txt,
        "points": points_txt,
        "sample": sample_txt,
        "cleanup": cleanup_txt,
        "backend": backend_txt,
        "recognizer_detail": recognizer_detail_txt,
        "progress": progress_txt,
        "interaction": interaction_txt,
        "confirm": confirm_txt,
        "candidate_rows": candidate_rows,
        "operator_candidate_rows": format_operator_candidate_rows(
            candidate_rows,
            status=state.preview_interaction_state or state.status,
            sample_ready=state.sample_ready,
        ),
        "result": format_candidate_result_summary(state.result_payload),
        "operator_result": format_operator_recognition_summary(state.result_payload),
        "operator_status": operator_action_status_text(state),
    }


def _basename(path):
    text = str(path or "").replace("\\", "/").rstrip("/")
    return text.split("/")[-1] if text else ""


def recognizer_status_texts(payload):
    """Compact Recognizer-card lines; no feature/debug fields in main UI."""
    if not isinstance(payload, dict):
        return {
            "mode": "Mode: unknown",
            "model": "Model: n/a",
            "status": "Status: unknown",
            "labels": "Labels: unknown",
        }
    backend = str(payload.get("backend") or payload.get("active_backend") or "unknown")
    mode_map = {
        "dtw_fallback_from_ocr_canvas": "DTW fallback",
        "dtw_trajectory_top3": "DTW",
        "dtw_template_bank": "DTW template bank",
    }
    mode = mode_map.get(backend, backend or "unknown")
    model = (
        str(payload.get("recognition_model_name") or "").strip()
        or _basename(payload.get("recognition_model_path") or payload.get("model_path") or "")
        or "n/a"
    )
    if payload.get("uncertain"):
        reason = str(payload.get("uncertainty_reason") or payload.get("reason") or "uncertain")
        status = "Uncertain / redraw (%s)" % reason
    else:
        status = str(payload.get("status") or "unknown")
    return {
        "mode": "Mode: %s" % mode,
        "model": "Model: %s" % model,
        "status": "Status: %s" % status,
        "labels": format_recognition_labels(payload.get("recognition_labels")),
    }
