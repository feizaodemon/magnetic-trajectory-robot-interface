"""Source text of the dashboard node together with its package mixin modules.

Static tests that assert on dashboard method text read it from here, so they do
not depend on whether a method lives in the node or in one of its mixins.
"""

from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
PACKAGE = REPO / "colmag_ros" / "src" / "colmag_ros"
DASHBOARD_FILES = (
    REPO / "colmag_ros" / "scripts" / "magnetic_trajectory_dashboard_node.py",
    PACKAGE / "dashboard_layout.py",
    PACKAGE / "dashboard_canvas_view.py",
    PACKAGE / "dashboard_sample_capture.py",
    PACKAGE / "dashboard_preview_controls.py",
    PACKAGE / "dashboard_ocr_panel.py",
    PACKAGE / "dashboard_gamepad.py",
)


def dashboard_source():
    return "\n".join(path.read_text() for path in DASHBOARD_FILES)
