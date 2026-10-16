"""Characterization tests for the preview-only semantic library.

They pin what the dashboard shows for every single-character label, so
removing catalog rows that no label can reach changes no displayed text.
"""

import string

from colmag_ros import semantic_library
from colmag_ros.semantic_library import (
    VISIBLE_TASKS,
    describe_candidate,
    preview_line,
    visible_task_for_label,
)

# label -> (task_id, title, safety_class, dispatchable_future)
EXPECTED = {
    '0': ('DRAW_HEXAGON', 'Draw Hexagon', 'SAFE', True),
    '1': ('FOLLOW_PATH', 'Follow Path', 'SAFE', False),
    '2': ('DRAW_HEXAGON', 'Draw Hexagon', 'SAFE', True),
    '3': ('FOLLOW_PATH', 'Follow Path', 'SAFE', False),
    '4': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    '5': ('DRAW_STAR', 'Draw Star', 'SAFE', False),
    '6': ('FOLLOW_PATH', 'Follow Path', 'SAFE', False),
    '7': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    '8': ('STACK_BLOCKS', 'Stack Blocks', 'REVIEW', False),
    '9': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'A': ('HOME_PREVIEW', 'Home (preview)', 'SAFE', False),
    'B': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'C': ('SOFT_DESCEND_PREVIEW', 'Soft Descend (preview)', 'SAFE', False),
    'D': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'E': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'F': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'G': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'H': ('HOME_PREVIEW', 'Home (preview)', 'SAFE', False),
    'I': ('INSPECT_OBJECT', 'Inspect Object', 'REVIEW', False),
    'J': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'K': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'L': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'M': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'N': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'O': ('ORBIT_SMALL', 'Small Orbit', 'SAFE', False),
    'P': ('PRESS_BUTTON', 'Press Button', 'REVIEW', False),
    'Q': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'R': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'S': ('STOP', 'Stop', 'CRITICAL', False),
    'T': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'U': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'V': ('HOVER_APPROACH', 'Hover Approach', 'SAFE', False),
    'W': ('WAVE', 'Wave', 'SAFE', False),
    'X': ('STOP_OR_CANCEL', 'Stop or Cancel', 'SAFE', False),
    'Y': ('UNKNOWN', 'Unknown', 'REVIEW', False),
    'Z': ('UNKNOWN', 'Unknown', 'REVIEW', False),
}


def test_every_single_character_label_keeps_its_visible_task():
    labels = string.digits + string.ascii_uppercase
    assert sorted(EXPECTED) == sorted(labels)
    for label in labels:
        d = describe_candidate({"label": label})
        assert (d["task_id"], d["title"], d["safety_class"], d["dispatchable_future"]) \
            == EXPECTED[label], label


def test_lookup_trims_and_uppercases_labels():
    assert visible_task_for_label("o")[0] == "ORBIT_SMALL"
    assert visible_task_for_label(" 2 ")[0] == "DRAW_HEXAGON"
    for blank in ("", None, "?", "UNKNOWN"):
        assert visible_task_for_label(blank)[0] == "UNKNOWN"


def test_symbol_semantics_wins_over_the_label_table():
    # symbol_semantics maps "O" to ORBIT_SMALL; the label table has no "O".
    assert "O" not in semantic_library._LABEL_TO_VISIBLE
    assert visible_task_for_label("O")[0] == "ORBIT_SMALL"


def test_every_catalog_row_is_reachable_from_some_label():
    reachable = {visible_task_for_label(label)[0]
                 for label in string.digits + string.ascii_uppercase}
    assert set(VISIBLE_TASKS) == reachable


def test_mapping_targets_exist_in_the_catalog():
    targets = set(semantic_library._SEMANTIC_TASK_TO_VISIBLE.values())
    targets |= set(semantic_library._LABEL_TO_VISIBLE.values())
    assert targets <= set(VISIBLE_TASKS)


def test_preview_line_text():
    assert preview_line({"label": "O", "confidence": 0.9}) == \
        "O -> Small Orbit [SAFE] (conf 0.90) — preview only"
    assert preview_line({"label": "2"}) == "2 -> Draw Hexagon [SAFE] — preview only"
    assert preview_line(None) == "? -> Unknown [REVIEW] — preview only"
