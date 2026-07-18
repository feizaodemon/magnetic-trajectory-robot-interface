"""Pure dashboard dwell, controller-mode, and confirmation decisions."""

import json
import time

from colmag_ros.dashboard_candidate_display import summarize_candidates


CONTROLLER_LAYOUT_RANK_CONFIRM = "rank_confirm"
CONTROLLER_LAYOUT_GAMEPAD = "gamepad"

GAMEPAD_MODE_MOTION = "MOTION_MODE"
GAMEPAD_MODE_BLOCKED = "BLOCKED_MODE"
GAMEPAD_MODE_DIGIT = "DIGIT_MODE"
GAMEPAD_MODE_CONFIRM_PENDING = "CONFIRM_PENDING_MODE"

GAMEPAD_ARROW_BUTTONS = ("U", "D", "L", "R")
GAMEPAD_FACE_BUTTONS = ("A", "B", "X", "C")
GAMEPAD_BUTTONS = GAMEPAD_ARROW_BUTTONS + GAMEPAD_FACE_BUTTONS
GAMEPAD_BUTTON_TO_LABEL = {
    "U": "V",
    "D": "6",
    "L": "1",
    "R": "3",
    "A": "S",
}
GAMEPAD_BUTTON_TO_INTENT = {
    "U": "MOVE_UP",
    "D": "MOVE_DOWN",
    "L": "MOVE_LEFT",
    "R": "MOVE_RIGHT",
    "A": "STOP",
    "B": "DIGIT_MODE",
    "X": "CLEAR",
    "C": "CONFIRM_RANK_1",
}


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


def normalize_controller_layout(value):
    if value is None:
        return CONTROLLER_LAYOUT_RANK_CONFIRM
    layout = str(value).strip().lower()
    if layout == CONTROLLER_LAYOUT_GAMEPAD:
        return CONTROLLER_LAYOUT_GAMEPAD
    return CONTROLLER_LAYOUT_RANK_CONFIRM


def build_virtual_button_rects(width, height, padding):
    del height
    x1 = padding + 14
    x2 = padding + 14 + 118
    y1 = padding + 14
    button_height = 42
    gap = 8
    buttons = []
    for index, label in enumerate(("RANK 1", "RANK 2", "RANK 3", "CLEAR")):
        top = y1 + index * (button_height + gap)
        button_id = "rank_%d" % (index + 1) if index < 3 else "clear"
        buttons.append(
            {
                "id": button_id,
                "label": label,
                "x1": x1,
                "y1": top,
                "x2": x2,
                "y2": top + button_height,
            }
        )
    return buttons


def _rect_from_center(button_id, label, center_x, center_y, size, kind="rect"):
    half = size / 2.0
    return {
        "id": button_id,
        "label": label,
        "x1": center_x - half,
        "y1": center_y - half,
        "x2": center_x + half,
        "y2": center_y + half,
        "kind": kind,
    }


def build_gamepad_button_rects(width, height, padding):
    size = max(42.0, min(58.0, (min(width, height) - 2.0 * padding) / 8.0))
    step = size + 8.0
    center_y = height / 2.0
    dpad_x = padding + 100.0
    face_x = width - padding - 100.0

    buttons = []
    for button_id, offset_x, offset_y in (
        ("U", 0.0, -step),
        ("D", 0.0, step),
        ("L", -step, 0.0),
        ("R", step, 0.0),
    ):
        buttons.append(
            _rect_from_center(
                button_id,
                button_id,
                dpad_x + offset_x,
                center_y + offset_y,
                size,
                kind="dpad",
            )
        )

    for button_id, offset_x, offset_y in (
        ("X", 0.0, -step),
        ("B", 0.0, step),
        ("A", -step, 0.0),
        ("C", step, 0.0),
    ):
        buttons.append(
            _rect_from_center(
                button_id,
                button_id,
                face_x + offset_x,
                center_y + offset_y,
                size,
                kind="face",
            )
        )
    return buttons


def build_controller_button_rects(layout, width, height, padding):
    if normalize_controller_layout(layout) == CONTROLLER_LAYOUT_GAMEPAD:
        return build_gamepad_button_rects(width, height, padding)
    return build_virtual_button_rects(width, height, padding)


def gamepad_button_to_label(button_id):
    return GAMEPAD_BUTTON_TO_LABEL.get(str(button_id).upper(), "")


def gamepad_button_to_intent(button_id):
    return GAMEPAD_BUTTON_TO_INTENT.get(str(button_id).upper(), "")


def update_controller_mode(
    previous_mode,
    activated_button,
    now,
    digit_mode_sec,
    has_candidates=False,
):
    mode = previous_mode or GAMEPAD_MODE_MOTION
    button_id = str(activated_button or "").upper()
    now = float(now)
    digit_mode_sec = max(float(digit_mode_sec), 0.0)
    changed = False
    should_confirm_rank_1 = False

    if mode == GAMEPAD_MODE_DIGIT and digit_mode_sec > 0.0:
        expires_at = now + digit_mode_sec
    else:
        expires_at = None

    if button_id == "A":
        mode = GAMEPAD_MODE_BLOCKED
        changed = True
    elif button_id == "B":
        mode = GAMEPAD_MODE_DIGIT
        expires_at = now + digit_mode_sec
        changed = True
    elif button_id == "X":
        mode = GAMEPAD_MODE_MOTION
        expires_at = None
        changed = True
    elif button_id == "C" and mode == GAMEPAD_MODE_CONFIRM_PENDING and has_candidates:
        should_confirm_rank_1 = True
        changed = True

    return {
        "mode": mode,
        "digit_started_at": now if button_id == "B" else None,
        "digit_expires_at": expires_at,
        "changed": changed,
        "should_confirm_rank_1": should_confirm_rank_1,
    }


def should_ignore_gamepad_button(mode, button_id, block_arrows_on_stop=True):
    button_id = str(button_id or "").upper()
    if not button_id:
        return True
    if (
        mode == GAMEPAD_MODE_BLOCKED
        and block_arrows_on_stop
        and button_id in GAMEPAD_ARROW_BUTTONS
    ):
        return True
    if mode == GAMEPAD_MODE_DIGIT and button_id not in ("B", "X"):
        return True
    if mode == GAMEPAD_MODE_CONFIRM_PENDING and button_id not in ("B", "C", "X"):
        return True
    return False


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


def build_gamepad_confirmed_label_payload(
    button_id,
    mode,
    candidate_payload=None,
    now_fn=time.time,
):
    button_id = str(button_id or "").upper()
    if should_ignore_gamepad_button(mode, button_id):
        return None, "button_ignored"

    if button_id in GAMEPAD_ARROW_BUTTONS + ("A",):
        label = gamepad_button_to_label(button_id)
        intent = gamepad_button_to_intent(button_id)
        if not label or not intent:
            return None, "button_has_no_label"
        return {
            "timestamp": now_fn(),
            "label": label,
            "confidence": 1.0,
            "confirmed": True,
            "confirmed_by": "magnetic_gamepad_dwell",
            "selected_button": button_id,
            "command_intent": intent,
            "controls_real_robot": False,
            "gazebo_only": True,
            "test_only": False,
            "source_topic": "/colmag/trajectory_2d",
        }, ""

    if button_id == "C" and mode == GAMEPAD_MODE_CONFIRM_PENDING:
        confirmed, reason = build_confirmed_label_payload(
            candidate_payload,
            1,
            now_fn=now_fn,
        )
        if confirmed is None:
            return None, reason
        confirmed["confirmed_by"] = "magnetic_gamepad_dwell"
        confirmed["selected_button"] = "C"
        confirmed["command_intent"] = "CONFIRM_RANK_1"
        return confirmed, ""

    return None, "button_has_no_confirmed_label"
