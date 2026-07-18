"""Pure real-board dashboard sample lifecycle decisions.

This module owns sample cleanup, drawing-zone state transitions, diagnostic
record construction, and capture payload construction.  It deliberately has no
ROS, Tk, confirmation, dispatch, Gazebo, or robot-control side effects.
"""

import math
import time
from pathlib import Path


DEFAULT_REAL_BOARD_SAMPLE_RECORDING_DIR = (
    "outputs/runtime_samples/real_board"
)


def optional_float(value):
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("", "none", "nan"):
            return None
        try:
            return float(text)
        except ValueError:
            return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def clean_stroke_points(points):
    return [point for point in points if point is not None]


def trajectory_input_inactive_reason(
    valid=True,
    z=None,
    z_min=None,
    z_max=None,
    gate_enabled=True,
):
    if gate_enabled and not bool(valid):
        return "valid=false"
    z_value = optional_float(z)
    if z_value is not None:
        min_value = optional_float(z_min)
        max_value = optional_float(z_max)
        if min_value is not None and z_value < min_value:
            return "z_below_min"
        if max_value is not None and z_value > max_value:
            return "z_above_max"
    return ""


def trajectory_input_is_active(
    valid=True,
    z=None,
    z_min=None,
    z_max=None,
    gate_enabled=True,
):
    return not trajectory_input_inactive_reason(
        valid,
        z,
        z_min,
        z_max,
        gate_enabled,
    )


def cleanup_drawing_sample_points(
    points,
    min_point_delta=0.0,
    max_points=0,
    enabled=True,
):
    """Deterministically thin the sample published to symbol_capture."""
    cleaned = clean_stroke_points(points)
    if not enabled:
        return list(cleaned)

    try:
        min_delta = max(0.0, float(min_point_delta))
    except (TypeError, ValueError):
        min_delta = 0.0
    try:
        max_count = max(0, int(max_points))
    except (TypeError, ValueError):
        max_count = 0

    if min_delta > 0.0 and cleaned:
        thinned = [cleaned[0]]
        for point in cleaned[1:]:
            previous = thinned[-1]
            distance = math.hypot(
                float(point[0]) - float(previous[0]),
                float(point[1]) - float(previous[1]),
            )
            if distance >= min_delta:
                thinned.append(point)
        if len(cleaned) > 1 and cleaned[-1] != thinned[-1]:
            thinned.append(cleaned[-1])
        cleaned = thinned

    if max_count > 0 and len(cleaned) > max_count:
        if max_count == 1:
            return [cleaned[0]]
        last = len(cleaned) - 1
        cleaned = [
            cleaned[int(index * last / float(max_count - 1))]
            for index in range(max_count)
        ]
    return list(cleaned)


def sample_cleanup_metadata(
    raw_points,
    cleaned_points,
    min_point_delta=0.0,
    max_points=0,
    enabled=True,
):
    try:
        min_delta = max(0.0, float(min_point_delta))
    except (TypeError, ValueError):
        min_delta = 0.0
    try:
        max_count = max(0, int(max_points))
    except (TypeError, ValueError):
        max_count = 0
    return {
        "sample_cleanup_enabled": bool(enabled),
        "sample_min_point_delta": min_delta,
        "sample_max_points": max_count,
        "raw_point_count": len(clean_stroke_points(raw_points)),
        "published_point_count": len(clean_stroke_points(cleaned_points)),
    }


def real_board_sample_recording_dir(path):
    base = Path(DEFAULT_REAL_BOARD_SAMPLE_RECORDING_DIR)
    text = str(path or "").strip().replace("\\", "/").strip("/")
    if not text:
        return base
    candidate = Path(text)
    if candidate.is_absolute():
        return base
    candidate_text = candidate.as_posix().strip("/")
    base_text = base.as_posix()
    if candidate_text == base_text or candidate_text.startswith(base_text + "/"):
        return candidate
    return base


def safe_sample_recording_name(sample_id):
    cleaned = []
    for character in str(sample_id or "unknown"):
        if character.isalnum() or character in ("-", "_", "."):
            cleaned.append(character)
        else:
            cleaned.append("_")
    name = "".join(cleaned).strip("._")
    return name or "unknown"


def build_real_board_sample_record(
    *,
    capture_payload,
    raw_points,
    cleaned_points,
    cleanup_metadata,
    candidate_payload=None,
):
    payload = dict(capture_payload or {})
    metadata = dict(cleanup_metadata or {})
    candidate = candidate_payload if isinstance(candidate_payload, dict) else None
    candidate_sample_id = candidate.get("sample_id") if candidate else None
    capture_sample_id = payload.get("sample_id")
    return {
        "record_schema": "m104_g3b_f6_real_board_sample_v1",
        "recorded_at": float(time.time()),
        "source": "magnetic_trajectory_dashboard_node",
        "sample_id": str(capture_sample_id or ""),
        "sequence_id": payload.get("sequence_id"),
        "raw_points": [
            [float(x), float(y)] for x, y in clean_stroke_points(raw_points)
        ],
        "cleaned_points": [
            [float(x), float(y)] for x, y in clean_stroke_points(cleaned_points)
        ],
        "capture_payload": payload,
        "cleanup": metadata,
        "latest_candidate_payload": candidate,
        "latest_candidate_may_be_stale": bool(
            candidate is not None
            and candidate_sample_id is not None
            and capture_sample_id is not None
            and str(candidate_sample_id) != str(capture_sample_id)
        ),
        "note": (
            "diagnostic record only; not used for recognition, training, "
            "thresholds, or dispatch"
        ),
    }


def update_drawing_zone_sample_state(
    state,
    point,
    inside_drawing_zone,
    input_active,
    inside_control_zone=False,
    append_point=True,
    force_freeze=False,
):
    """Track DRAWING_ACTIVE -> PEN_UP_PENDING -> SAMPLE_FROZEN."""
    state = state or {}
    active_points = list(state.get("active_points") or [])
    frozen = state.get("frozen_points")
    frozen_points = list(frozen) if frozen else []
    was_inside = bool(state.get("was_inside", False))
    phase = str(state.get("phase") or "IDLE")

    if not input_active:
        if was_inside and active_points and active_points[-1] is not None:
            active_points.append(None)
        if not active_points and not frozen_points:
            phase = "IDLE"
        else:
            phase = "SAMPLE_FROZEN" if frozen_points else "PEN_UP_PENDING"
        return {
            "active_points": active_points,
            "frozen_points": frozen_points,
            "was_inside": False,
            "phase": phase,
        }

    if (force_freeze or inside_control_zone) and active_points and not frozen_points:
        frozen_points = list(active_points)
        phase = "SAMPLE_FROZEN"
    elif inside_drawing_zone:
        if append_point and not frozen_points:
            active_points.append(point)
        was_inside = True
        phase = "SAMPLE_FROZEN" if frozen_points else "DRAWING_ACTIVE"
    else:
        if was_inside and active_points and active_points[-1] is not None:
            active_points.append(None)
        was_inside = False
        phase = "SAMPLE_FROZEN" if frozen_points else "PEN_UP_PENDING"

    return {
        "active_points": active_points,
        "frozen_points": frozen_points,
        "was_inside": was_inside,
        "phase": phase,
    }


def build_dashboard_drawing_zone_capture_payload(
    points,
    sample_id,
    sequence_id,
    timestamp,
    source_topic,
    drawing_zone=None,
    raw_points=None,
    cleanup_metadata=None,
):
    metadata = dict(cleanup_metadata or {})
    source_points = raw_points if raw_points is not None else points
    raw_count = len(clean_stroke_points(source_points))
    published_count = len(clean_stroke_points(points))
    payload = {
        "timestamp": float(timestamp),
        "sample_id": str(sample_id),
        "sequence_id": int(sequence_id),
        "points": [[float(x), float(y)] for x, y in points],
        "duration_sec": 0.0,
        "source": "magnetic_trajectory_dashboard_node",
        "source_topic": source_topic,
        "capture_mode": "dashboard_drawing_zone",
        "controls_excluded_from_sample": True,
        "controls_real_robot": False,
        "test_only": False,
        "raw_point_count": int(metadata.get("raw_point_count", raw_count)),
        "published_point_count": int(
            metadata.get("published_point_count", published_count)
        ),
        "sample_cleanup_enabled": bool(
            metadata.get("sample_cleanup_enabled", False)
        ),
        "sample_min_point_delta": float(
            metadata.get("sample_min_point_delta", 0.0)
        ),
        "sample_max_points": int(metadata.get("sample_max_points", 0)),
    }
    if drawing_zone:
        payload["drawing_zone_canvas_rect"] = {
            "x1": float(drawing_zone["x1"]),
            "y1": float(drawing_zone["y1"]),
            "x2": float(drawing_zone["x2"]),
            "y2": float(drawing_zone["y2"]),
        }
    return payload
