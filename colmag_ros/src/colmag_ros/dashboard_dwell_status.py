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


def format_integrated_dwell_status_texts(
    integrated_confirm_enabled,
    current_button_id,
    dwell_progress,
    dwell_confirm_sec,
):
    button_id = current_button_id or "-"
    progress = float(dwell_progress or 0.0)
    dwell_sec = float(dwell_confirm_sec)
    enabled = "enabled" if integrated_confirm_enabled else "disabled"
    return {
        "dwell": "Integrated dwell confirm: %s | Hover: %s | Dwell: %.1f / %.1f s" % (
            enabled,
            button_id,
            progress * dwell_sec,
            dwell_sec,
        ),
    }
