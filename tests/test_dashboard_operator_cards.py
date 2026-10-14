"""Operator cards: status tone, recognition tile and meter, robot task card, TELEOP view."""

from colmag_ros import dashboard_theme as theme
from colmag_ros.dashboard_texts import (
    TELEOP_MOUSE_HINT,
    TELEOP_NOTICE_TITLE,
    TELEOP_PAD_CAPTION,
    TELEOP_PANEL_TITLE,
    TELEOP_STATUS,
)
from dashboard_harness import DashboardHarness

CONFIRM_PARAMS = {
    "integrated_dashboard_confirm_enabled": True,
    "publish_confirmed_label": True,
}


def _texts(canvas):
    return [options["text"] for options in canvas.drawn("text")]


def test_candidates_fill_the_symbol_tile_meter_and_status_tone(monkeypatch):
    harness = DashboardHarness(monkeypatch, candidate_preview_enabled=True)
    node = harness.node
    assert _texts(node.recognition_tile) == ["?"]
    assert node.confidence_meter.drawn("polygon") == []

    harness.send_sample_and_candidates()

    assert _texts(node.recognition_tile) == ["2"]
    track, bar = node.confidence_meter.drawn("polygon")
    assert track["fill"] == theme.BORDER
    # SAMPLE_CANDIDATES carries no "accepted" flag, so the result reads as not accepted.
    assert bar["fill"] == theme.tone_colors("warning")[0]
    assert node.status_tone_bar.options["bg"] == theme.tone_colors("accent")[0]
    assert node.workflow_stage == "review"


def test_robot_card_shows_task_and_execution_state_only_with_a_confirm_path(monkeypatch):
    preview = DashboardHarness(monkeypatch, candidate_preview_enabled=True)
    assert "Robot task" not in preview.card_titles()
    assert preview.node.robot_state_dot is None

    harness = DashboardHarness(monkeypatch, demo_input_mode="ocr_canvas", **CONFIRM_PARAMS)
    node = harness.node
    assert harness.card_titles() == ["Status", "Progress", "Recognition", "Robot task"]
    assert node.task_var.get() == "No task sent yet"

    harness.send(node.config.task_command_topic, {"task": "HEXAGON_TRAJECTORY"})
    harness.send(node.config.demo_task_state_topic, {"status": "running", "phase": "waypoint_2"})
    assert node.task_var.get() == "Task: Hexagon trajectory"
    assert node.gazebo_var.get() == "Running · waypoint 2"
    assert node.robot_state_dot.drawn("oval")[-1]["fill"] == theme.tone_colors("info")[0]
    assert node.task_var in harness.shown_variables()
    assert node.gazebo_var in harness.shown_variables()


def _lit_keycaps(node):
    return [item for item in node.canvas.drawn("polygon") if item.get("fill") == theme.ACCENT]


def test_mouse_teleop_view_swaps_stage_and_cards_and_pauses_mouse_actions(monkeypatch):
    harness = DashboardHarness(
        monkeypatch, demo_input_mode="ocr_canvas", control_mode_enabled=True, **CONFIRM_PARAMS)
    node = harness.node
    workflow, recognition = node.task_flow_cards
    idle_status = node.mouse_operator_status_var.get()
    assert harness.card_titles() == ["Status", "Progress", "Recognition", "Teleoperation", "Robot task"]
    assert node.teleop_card.pack_options is None

    harness.send(node.config.control_mode_topic, "TELEOP")
    assert node.mouse_operator_status_var.get() == TELEOP_STATUS
    assert node.ocr_hint_var.get() == TELEOP_MOUSE_HINT
    assert node.ocr_title_label.options["text"] == TELEOP_PANEL_TITLE
    assert node.status_tone_bar.options["bg"] == theme.tone_colors("info")[0]
    assert node.control_mode_chip.options["bg"] == theme.tone_colors("info")[0]
    assert node.teleop_card.pack_options["after"] is node.status_card
    assert workflow.pack_options is None and recognition.pack_options is None
    assert TELEOP_PAD_CAPTION in harness.canvas_texts()
    assert {"W", "A", "S", "D", "Q", "E"} <= set(harness.canvas_texts())
    node._handle_ocr_action("B")
    assert node.ocr_panel_state == "WAITING"

    harness.send(node.config.cartesian_input_topic,
                 {"engaged": True, "x": 1.0, "y": 0.0, "z": 0.0, "input_mode": "keyboard_cartesian"})
    harness.send(node.config.cartesian_teleop_state_topic,
                 {"state": "COMMAND_PUBLISHED", "reason": "bounded_cartesian_target",
                  "simulation_commands_enabled": True, "hardware_active": False})
    assert node.teleop_headline_var.get() == "Moving +X"
    assert node.teleop_backend_var.get() == "Simulation commands · hardware inactive"
    assert len(_lit_keycaps(node)) == 1

    harness.send(node.config.cartesian_teleop_state_topic,
                 {"state": "PAUSED", "reason": "stale_cartesian_input"})
    assert node.teleop_headline_var.get() == "Paused"
    assert node.teleop_detail_var.get() == "Stale cartesian input"
    assert _lit_keycaps(node) == []

    harness.send(node.config.control_mode_topic, "TASK")
    assert node.mouse_operator_status_var.get() == idle_status
    assert node.control_mode_chip.options["bg"] == theme.HEADER_CHIP
    assert node.teleop_card.pack_options is None
    assert workflow.pack_options["after"] is node.status_card
    assert recognition.pack_options["after"] is workflow
    assert TELEOP_PAD_CAPTION not in harness.canvas_texts()
    node._handle_ocr_action("B")
    assert node.ocr_panel_state == "DRAWING"


def test_board_teleop_keeps_the_trajectory_stage_under_a_notice(monkeypatch):
    harness = DashboardHarness(monkeypatch, candidate_preview_enabled=True, control_mode_enabled=True)
    node = harness.node
    assert node.teleop_card is None

    harness.send(node.config.control_mode_topic, "TELEOP")
    assert node.traj_operator_status_var.get() == TELEOP_STATUS
    assert TELEOP_NOTICE_TITLE in harness.canvas_texts()
    assert TELEOP_PAD_CAPTION not in harness.canvas_texts()
