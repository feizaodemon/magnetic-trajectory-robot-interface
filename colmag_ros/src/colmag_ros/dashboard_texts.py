"""Default header, panel, and footer texts for the trajectory dashboard."""

TRAJECTORY_MODE = "trajectory"
HEADER_TITLE = "COLMAG Magnetic Trajectory Dashboard"

# TELEOP blocks new task dispatch. The mouse profile also pauses drawing; the
# board profile keeps drawing and recognition running.
TELEOP_STATUS = "TELEOP: Cartesian input drives the robot"
TELEOP_MOUSE_HINT = "Drawing is paused in TELEOP. Switch to TASK to draw."
TELEOP_PANEL_TITLE = "Cartesian Teleop Input"
TELEOP_PAD_CAPTION = "Keys held in the teleop terminal"
TELEOP_NOTICE_TITLE = "TELEOP active"
TELEOP_NOTICE_BODY = (
    "Cartesian input drives the robot.\n"
    "Task dispatch is paused.\n"
    "Switch to TASK to send symbols."
)


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
