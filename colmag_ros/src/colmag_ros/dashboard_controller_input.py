"""Pure dashboard dwell and confirmation decisions."""

import json
import time

from colmag_ros.dashboard_candidate_display import summarize_candidates


def cancelled_preview_hover_state():
    return {
        "current_button_id": "",
        "started_at": None,
        "dwell_progress": 0.0,
        "activated_button_id": "",
    }


def update_dwell_state(previous_state, current_button_id, now, dwell_sec):
    previous_state = previous_state or {}
    dwell_sec = max(float(dwell_sec), 1e-9)
    current_button_id = current_button_id or ""

    if not current_button_id:
        return cancelled_preview_hover_state()

    if previous_state.get("current_button_id") == current_button_id:
        started_at = previous_state.get("started_at")
        if started_at is None:
            started_at = now
    else:
        started_at = now

    progress = max(0.0, min(1.0, (now - started_at) / dwell_sec))
    activated = current_button_id if progress >= 1.0 else ""
    return {
        "current_button_id": current_button_id,
        "started_at": started_at,
        "dwell_progress": progress,
        "activated_button_id": activated,
    }


def preview_button_availability(has_sample, has_candidate, has_clearable_content):
    """Return the existing visible actions that can do useful work now."""
    return {
        "B": bool(has_sample),
        "C": bool(has_candidate),
        "A": bool(has_candidate),
        "X": bool(has_clearable_content),
    }


def _extract_candidate_by_rank(payload, selected_rank):
    for rank, label, confidence in summarize_candidates(payload):
        if rank == selected_rank:
            return {"rank": rank, "label": label, "confidence": confidence}
    return None


def candidate_payload_key(payload):
    if not isinstance(payload, dict):
        return ""
    sequence_id = payload.get("sequence_id", "")
    sample_id = payload.get("sample_id", "")
    if sequence_id != "" or sample_id != "":
        return "%s:%s" % (sequence_id, sample_id)
    return json.dumps(payload.get("candidates", []), sort_keys=True)


def should_suppress_repeated_confirm(last_key, payload, selected_rank):
    key = "%s:rank_%d" % (candidate_payload_key(payload), int(selected_rank))
    return key == last_key


def build_confirmed_label_payload(candidate_payload, selected_rank, now_fn=time.time):
    if not isinstance(candidate_payload, dict):
        return None, "payload_not_object"
    try:
        selected_rank = int(selected_rank)
    except (TypeError, ValueError):
        return None, "invalid_rank"
    if selected_rank not in (1, 2, 3):
        return None, "invalid_rank"

    candidate = _extract_candidate_by_rank(candidate_payload, selected_rank)
    if candidate is None:
        return None, "rank_not_available"

    output = {}
    for key in ("backend", "feature_mode", "sample_id", "sequence_id", "candidates"):
        if key in candidate_payload:
            output[key] = candidate_payload[key]
    output.update(
        {
            "timestamp": now_fn(),
            "label": candidate["label"],
            "confidence": candidate["confidence"],
            "confirmed": True,
            "confirmed_by": "magnetic_dashboard_dwell",
            "selected_rank": candidate["rank"],
            "controls_real_robot": False,
            "test_only": False,
            "source_topic": "/colmag/symbol_capture",
        }
    )
    return output, ""
