"""Package-level dwell/status text helpers for the dashboard."""


def normalize_interaction_profile(value, demo_input_mode="trajectory"):
    default = "real_board" if demo_input_mode == "trajectory" else "mouse"
    profile = str(value or default).strip().lower()
    return profile if profile in ("real_board", "mouse") else default


def format_interaction_instruction(profile):
    if profile == "real_board":
        return "Flip while moving; re-engage to draw or hover a control."
    return "Hover B to start drawing; release the stroke, then use C / A / X."


def format_ocr_canvas_dwell_texts(panel_state, hover_button, dwell_progress, dwell_confirm_sec):
    button_id = hover_button or "-"
    progress = float(dwell_progress or 0.0)
    return {
        "dwell": (
            "Mode: OCR_CANVAS | State: %s | Hover: %s | Dwell: %.0f%% | "
            "Hover control: enabled | Dwell confirm: %.1fs"
        ) % (
            panel_state,
            button_id,
            progress * 100.0,
            float(dwell_confirm_sec),
        ),
        "controller": "Mode: OCR_CANVAS | State: %s" % panel_state,
        "hover": "Hover: %s | Dwell: %.0f%%" % (button_id, progress * 100.0),
    }


def format_trajectory_preview_interaction_text(
    preview_interaction_state,
    hover_source,
    hover_button,
    dwell_progress,
    hover_progress_enabled=True,
):
    active_button = hover_button or "-"
    source = hover_source or "none"
    progress = float(dwell_progress or 0.0) if hover_progress_enabled else 0.0
    return "Interaction: %s | Source: %s | Active: %s | Dwell: %.0f%%" % (
        preview_interaction_state,
        source,
        active_button,
        progress * 100.0,
    )


def format_trajectory_preview_hint_text(use_canvas_reachable_controls):
    if use_canvas_reachable_controls:
        return format_interaction_instruction("real_board")
    return "Move the board cursor (or mouse) onto X Clear, A Reject, B Recognize, C Confirm"


def format_integrated_dwell_status_texts(
    integrated_confirm_enabled,
    controller_layout,
    current_button_id,
    dwell_progress,
    dwell_confirm_sec,
    controller_mode,
    last_command_intent,
    gamepad_layout="gamepad",
):
    button_id = current_button_id or "-"
    progress = float(dwell_progress or 0.0)
    dwell_sec = float(dwell_confirm_sec)
    enabled = "enabled" if integrated_confirm_enabled else "disabled"
    mode_text = "Controller: %s" % controller_layout
    if controller_layout == gamepad_layout:
        mode_text = "Controller: %s | Mode: %s" % (controller_layout, controller_mode)
    return {
        "dwell": "Integrated dwell confirm: %s | Layout: %s | Hover: %s | Dwell: %.1f / %.1f s" % (
            enabled,
            controller_layout,
            button_id,
            progress * dwell_sec,
            dwell_sec,
        ),
        "controller": mode_text,
        "hover": "Hover: %s | Last intent: %s" % (button_id, last_command_intent or "-"),
    }
