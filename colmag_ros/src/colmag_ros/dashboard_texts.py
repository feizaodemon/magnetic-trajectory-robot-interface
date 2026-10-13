"""Default header, panel, and footer texts for the trajectory dashboard."""

TRAJECTORY_MODE = "trajectory"


def default_header_title(demo_input_mode):
    return "COLMAG Magnetic Trajectory Dashboard"


def default_header_subtitle(demo_input_mode):
    if demo_input_mode == "ocr_canvas":
        return "Draw, review the recognition result, then confirm or clear"
    if demo_input_mode == TRAJECTORY_MODE:
        return "Draw, recognize, review the result, then confirm or clear"
    return "Magnetic trajectory → Symbol recognition → Human confirmation → Gazebo task"


def default_main_panel_title(demo_input_mode):
    if demo_input_mode == "ocr_canvas":
        return "Mouse Trajectory Input"
    if demo_input_mode == TRAJECTORY_MODE:
        return "Live Magnetic Trajectory"
    return "Symbol Recognition Preview"


def default_safety_badge(demo_input_mode):
    return ""


def default_footer_text(demo_input_mode):
    return ""
