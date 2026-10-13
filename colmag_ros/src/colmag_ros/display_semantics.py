"""Display-only semantic labels for the 8-symbol recognition display.

This module is intentionally small and ROS-free. It does not dispatch tasks,
publish labels, or touch Gazebo / robot routes. The strings here are for
recognition display only.
"""

C2C3_DISPLAY_SEMANTICS = {
    "1": {
        "display_name": "MOVE_LEFT",
        "note": "Existing safe Gazebo subset member; display-only in C2C3.",
    },
    "2": {
        "display_name": "HEXAGON_TRAJECTORY",
        "note": "Existing safe Gazebo subset member; display-only in C2C3.",
    },
    "3": {
        "display_name": "MOVE_RIGHT",
        "note": "Existing safe Gazebo subset member; display-only in C2C3.",
    },
    "V": {
        "display_name": "HOVER_APPROACH",
        "note": "Future planning metadata only; no execution binding.",
    },
    "O": {
        "display_name": "ORBIT_SMALL / CIRCLE_PREVIEW",
        "note": "Future planning metadata only; no execution binding.",
    },
    "X": {
        "display_name": "STOP_OR_CANCEL",
        "note": "Future planning metadata only; no execution binding.",
    },
    "A": {
        "display_name": "HOME_OR_READY",
        "note": "Future planning metadata only; no execution binding.",
    },
    "C": {
        "display_name": "COMPLIANT_DEMO_SAFE / SOFT_DESCEND_PREVIEW",
        "note": "Future planning metadata only; no execution binding.",
    },
}


def display_semantics_for_label(label):
    """Return display-only semantics for a C2C3 label, or None."""
    return C2C3_DISPLAY_SEMANTICS.get(str(label or "").strip())


def display_label_for_candidate(candidate):
    """Return the preferred dashboard display text for a candidate.

    Unknown labels fall back to the candidate's existing display_name. No task or
    command metadata is produced here.
    """
    if not isinstance(candidate, dict):
        return ""
    semantics = display_semantics_for_label(candidate.get("label", ""))
    if semantics:
        return semantics["display_name"]
    return str(candidate.get("display_name", "") or "")
