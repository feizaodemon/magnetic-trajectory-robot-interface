import pytest

from colmag_ros.preview_interaction import PreviewInteraction


def _values(preview):
    return preview.candidate_status, preview.state, preview.confirmed_label


@pytest.mark.parametrize("status, state", [
    ("ready", "candidates_ready"),
    ("collecting", "collecting"),
    ("unavailable", "error"),
    ("error", "error"),
    ("uncertain", "uncertain"),
])
def test_recognized_maps_recognizer_status_to_state(status, state):
    preview = PreviewInteraction()
    preview.recognized(status)
    assert (preview.candidate_status, preview.state) == (status, state)


@pytest.mark.parametrize("has_candidates, uncertain, expected", [
    (True, True, ("ready", "external_candidates_ready")),
    (False, True, ("uncertain", "external_candidate_uncertain")),
    (False, False, ("none", "external_no_candidate")),
])
def test_external_candidates(has_candidates, uncertain, expected):
    preview = PreviewInteraction()
    preview.external_candidates(has_candidates=has_candidates, uncertain=uncertain)
    assert (preview.candidate_status, preview.state) == expected


def test_confirm_then_reject_and_clear_drop_the_label():
    preview = PreviewInteraction()
    preview.recognized("ready")
    assert preview.confirm("2", has_candidates=True) is True
    assert _values(preview) == ("ready", "preview_confirmed", "2")

    preview.reject()
    assert _values(preview) == ("rejected", "rejected", "")

    preview.confirm("3", has_candidates=True)
    preview.clear()
    assert _values(preview) == ("idle", "cleared", "")


def test_confirm_without_label_falls_back_and_keeps_status():
    preview = PreviewInteraction()
    preview.recognized("collecting")
    assert preview.confirm("", has_candidates=False) is False
    assert _values(preview) == ("collecting", "idle", "")
    assert preview.confirm("", has_candidates=True) is False
    assert preview.state == "ready"


def test_drawing_and_recording_transitions_keep_a_confirmed_label():
    preview = PreviewInteraction()
    preview.confirm("2", has_candidates=True)
    for transition, expected in (
        (preview.new_drawing, ("idle", "new_drawing_started")),
        (preview.too_few_points, ("collecting", "too_few_drawing_points")),
        (preview.capture_published, ("recognizing", "symbol_capture_published")),
        (preview.recording, ("recording", "recording")),
        (preview.recording_done, ("recording_done", "recording_done")),
        (preview.recognizing, ("recognizing", "recognizing")),
    ):
        transition()
        assert _values(preview) == expected + ("2",)
