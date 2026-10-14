"""Operator cards: status tone, recognition tile and meter, robot task card."""

from colmag_ros import dashboard_theme as theme
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
