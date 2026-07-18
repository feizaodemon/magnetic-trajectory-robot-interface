from colmag_ros.scripts.dashboard_dwell_status import (
    format_interaction_instruction,
    format_integrated_dwell_status_texts,
    format_ocr_canvas_dwell_texts,
    format_trajectory_preview_hint_text,
    format_trajectory_preview_interaction_text,
    normalize_interaction_profile,
)


def test_ocr_canvas_dwell_texts_preserve_existing_strings():
    texts = format_ocr_canvas_dwell_texts(
        panel_state="DRAWING",
        hover_button="B",
        dwell_progress=0.42,
        dwell_confirm_sec=2.0,
    )

    assert texts == {
        "dwell": "Mode: OCR_CANVAS | State: DRAWING | Hover: B | Dwell: 42% | Hover control: enabled | Dwell confirm: 2.0s",
        "controller": "Mode: OCR_CANVAS | State: DRAWING",
        "hover": "Hover: B | Dwell: 42%",
    }


def test_ocr_canvas_dwell_texts_use_dash_for_no_hover():
    texts = format_ocr_canvas_dwell_texts(
        panel_state="WAITING",
        hover_button="",
        dwell_progress=0.0,
        dwell_confirm_sec=1.5,
    )

    assert texts["dwell"] == (
        "Mode: OCR_CANVAS | State: WAITING | Hover: - | Dwell: 0% | "
        "Hover control: enabled | Dwell confirm: 1.5s"
    )
    assert texts["hover"] == "Hover: - | Dwell: 0%"


def test_trajectory_preview_interaction_text_preserves_enabled_progress():
    assert format_trajectory_preview_interaction_text(
        preview_interaction_state="ready",
        hover_source="board",
        hover_button="C",
        dwell_progress=0.42,
        hover_progress_enabled=True,
    ) == "Interaction: ready | Source: board | Active: C | Dwell: 42%"


def test_trajectory_preview_interaction_text_zeroes_disabled_progress_and_defaults():
    assert format_trajectory_preview_interaction_text(
        preview_interaction_state="collecting",
        hover_source="",
        hover_button="",
        dwell_progress=0.75,
        hover_progress_enabled=False,
    ) == "Interaction: collecting | Source: none | Active: - | Dwell: 0%"


def test_trajectory_preview_hint_text_preserves_layout_specific_strings():
    assert format_trajectory_preview_hint_text(True) == (
        "Flip while moving; re-engage to draw or hover a control."
    )
    assert format_trajectory_preview_hint_text(False) == (
        "Move the board cursor (or mouse) onto X Clear, A Reject, B Recognize, C Confirm"
    )


def test_interaction_profile_instructions_are_source_accurate():
    assert normalize_interaction_profile(None, "trajectory") == "real_board"
    assert normalize_interaction_profile(None, "ocr_canvas") == "mouse"
    assert format_interaction_instruction("real_board") == (
        "Flip while moving; re-engage to draw or hover a control."
    )
    mouse = format_interaction_instruction("mouse")
    assert mouse == "Hover B to start drawing; release the stroke, then use C / A / X."
    assert "Flip" not in mouse
    assert "tracking_mz" not in mouse


def test_integrated_rank_confirm_status_texts_preserve_existing_strings():
    texts = format_integrated_dwell_status_texts(
        integrated_confirm_enabled=True,
        controller_layout="rank_confirm",
        current_button_id="rank_1",
        dwell_progress=0.5,
        dwell_confirm_sec=2.0,
        controller_mode="MOTION_MODE",
        last_command_intent="MOVE_LEFT",
        gamepad_layout="gamepad",
    )

    assert texts == {
        "dwell": "Integrated dwell confirm: enabled | Layout: rank_confirm | Hover: rank_1 | Dwell: 1.0 / 2.0 s",
        "controller": "Controller: rank_confirm",
        "hover": "Hover: rank_1 | Last intent: MOVE_LEFT",
    }


def test_integrated_gamepad_status_texts_preserve_mode_and_fallbacks():
    texts = format_integrated_dwell_status_texts(
        integrated_confirm_enabled=False,
        controller_layout="gamepad",
        current_button_id="",
        dwell_progress=0.25,
        dwell_confirm_sec=4.0,
        controller_mode="DIGIT_MODE",
        last_command_intent="",
        gamepad_layout="gamepad",
    )

    assert texts == {
        "dwell": "Integrated dwell confirm: disabled | Layout: gamepad | Hover: - | Dwell: 1.0 / 4.0 s",
        "controller": "Controller: gamepad | Mode: DIGIT_MODE",
        "hover": "Hover: - | Last intent: -",
    }
