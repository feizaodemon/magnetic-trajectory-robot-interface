from pathlib import Path

from colmag_ros import dashboard_controller_input as controls


def test_dwell_state_preserves_activation_and_reset_contract():
    started = controls.update_dwell_state({}, "rank_1", 10.0, 2.0)
    assert started == {
        "current_button_id": "rank_1",
        "started_at": 10.0,
        "dwell_progress": 0.0,
        "activated_button_id": "",
    }
    activated = controls.update_dwell_state(started, "rank_1", 12.0, 2.0)
    assert activated["dwell_progress"] == 1.0
    assert activated["activated_button_id"] == "rank_1"
    assert controls.cancelled_preview_hover_state()["current_button_id"] == ""


def test_controller_mode_and_gamepad_confirmation_preserve_safety_flags():
    transition = controls.update_controller_mode(
        controls.GAMEPAD_MODE_MOTION,
        "B",
        10.0,
        3.0,
    )
    assert transition["mode"] == controls.GAMEPAD_MODE_DIGIT
    assert transition["digit_expires_at"] == 13.0

    payload, reason = controls.build_gamepad_confirmed_label_payload(
        "A",
        controls.GAMEPAD_MODE_MOTION,
        now_fn=lambda: 123.0,
    )
    assert reason == ""
    assert payload["label"] == "S"
    assert payload["command_intent"] == "STOP"
    assert payload["controls_real_robot"] is False
    assert payload["gazebo_only"] is True


def test_rank_confirmation_preserves_candidate_identity_and_schema():
    candidate_payload = {
        "backend": "dtw_template_bank",
        "sample_id": "sample-1",
        "sequence_id": 7,
        "candidates": [
            {"rank": 1, "label": "2", "confidence": 0.9},
            {"rank": 2, "label": "1", "confidence": 0.5},
        ],
    }
    confirmed, reason = controls.build_confirmed_label_payload(
        candidate_payload,
        2,
        now_fn=lambda: 5.0,
    )

    assert reason == ""
    assert confirmed["label"] == "1"
    assert confirmed["selected_rank"] == 2
    assert confirmed["confirmed_by"] == "magnetic_dashboard_dwell"
    assert confirmed["controls_real_robot"] is False
    assert controls.candidate_payload_key(candidate_payload) == "7:sample-1"


def test_controller_input_module_has_no_runtime_side_effect_dependencies():
    source = Path(controls.__file__).read_text()
    for forbidden in (
        "import rospy",
        "import tkinter",
        "Publisher(",
        "Subscriber(",
        "FollowJointTrajectory",
        "franka_control",
    ):
        assert forbidden not in source


def test_preview_button_availability_matches_real_action_guards():
    assert controls.preview_button_availability(False, False, False) == {
        "B": False, "C": False, "A": False, "X": False,
    }
    assert controls.preview_button_availability(True, False, True) == {
        "B": True, "C": False, "A": False, "X": True,
    }
    assert controls.preview_button_availability(True, True, True) == {
        "B": True, "C": True, "A": True, "X": True,
    }
