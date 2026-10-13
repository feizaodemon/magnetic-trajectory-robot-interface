from colmag_ros.execution_semantics import C4_LABEL_TO_TASK
from colmag_ros.symbol_semantics import get_symbol_semantics, enrich_candidate_with_semantics

def test_symbol_semantics_known_labels():
    for label, expected_task in [
        ("1", "MOVE_LEFT"),
        ("2", "HEXAGON_TRAJECTORY"),
        ("A", "HOME_OR_READY"),
        ("X", "STOP_OR_CANCEL"),
        ("S", "STOP"),
    ]:
        sem = get_symbol_semantics(label)
        assert sem is not None
        assert sem["task"] == expected_task
        assert "display_name" in sem
        assert "command_intent" in sem

def test_c4_labels_use_the_c4_dispatcher_task():
    for label, task in C4_LABEL_TO_TASK.items():
        assert get_symbol_semantics(label)["task"] == task

def test_symbol_semantics_unknown_label():
    assert get_symbol_semantics("UNKNOWN") is None

def test_enrich_easyocr_candidate():
    candidate = {
        "label": "2",
        "confidence": 0.99,
        "rank": 1,
        "source": "easyocr"
    }
    enriched = enrich_candidate_with_semantics(candidate)
    assert enriched["task"] == "HEXAGON_TRAJECTORY"
    assert enriched["display_name"] == "Hexagon Trajectory"
    assert enriched["label"] == "2"

def test_existing_candidate_schema_compatible():
    # consumers expecting just label/confidence/rank should be fine
    # because enrich mutates and adds fields, without deleting or renaming existing ones
    candidate = {
        "label": "A",
        "confidence": 0.8,
        "rank": 1,
    }
    enriched = enrich_candidate_with_semantics(candidate)
    assert enriched["label"] == "A"
    assert enriched["confidence"] == 0.8
    assert enriched["rank"] == 1
    assert "task" in enriched
