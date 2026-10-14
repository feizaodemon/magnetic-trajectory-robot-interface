"""TELEOP view model: Cartesian adapter state and input as card text and pad keys.

ROS-free and Tk-free. The dashboard only displays this; it publishes nothing
on the TELEOP route.
"""

from colmag_ros.cartesian_keyboard_input import KEY_DIRECTIONS

AXES = ("x", "y", "z")

# Adapter state -> (headline, tone). Unknown states are shown as-is, neutral.
STATE_VIEWS = {
    "COMMAND_PUBLISHED": ("Moving", "success"),
    "STOPPING": ("Stopping", "neutral"),
    "WAITING_FOR_INPUT": ("Waiting for input", "neutral"),
    "PAUSED": ("Paused", "warning"),
    "REJECTED": ("Target rejected", "danger"),
    "DRY_RUN_TARGET": ("Dry run: execution gates closed", "warning"),
    "TASK_MODE": ("TASK owns the robot", "neutral"),
    "WAITING_FOR_MODE": ("Waiting for control mode", "neutral"),
}

NO_STATE_VIEW = ("No adapter state yet", "Is the Cartesian teleop adapter running?", "neutral")


def input_direction(payload):
    """The engaged ``(x, y, z)`` direction of one input message, or ``None``."""
    if not isinstance(payload, dict) or payload.get("engaged") is not True:
        return None
    try:
        vector = tuple(float(payload.get(axis, 0.0)) for axis in AXES)
    except (TypeError, ValueError):
        return None
    return vector if any(vector) else None


def direction_label(vector):
    """``"+X"``, ``"-Y"``, ``"+X +Z"`` ...; empty for ``None``."""
    if vector is None:
        return ""
    return " ".join(
        "%s%s" % ("+" if value > 0 else "-", axis.upper())
        for axis, value in zip(AXES, vector)
        if value
    )


def pad_keys():
    """``[(key letter, direction, axis label)]`` for the single-letter keyboard keys."""
    return [
        (key.upper(), vector, direction_label(vector))
        for key, vector in KEY_DIRECTIONS.items()
        if len(key) == 1
    ]


def teleop_state_view(state_payload, direction=None):
    """``(headline, detail, tone)`` for the TELEOP card."""
    if not isinstance(state_payload, dict) or not state_payload.get("state"):
        return NO_STATE_VIEW
    state = str(state_payload["state"])
    headline, tone = STATE_VIEWS.get(state, (state.replace("_", " ").capitalize(), "neutral"))
    if state == "COMMAND_PUBLISHED":
        if direction is not None:
            headline = "Moving %s" % direction_label(direction)
        detail = "Bounded Cartesian target sent"
    elif state == "STOPPING":
        detail = "Key released, braking to rest"
    elif state == "WAITING_FOR_INPUT":
        detail = (
            "Hold a direction key in the teleop terminal"
            if state_payload.get("input_mode") == "normalized_cartesian"
            else "Waiting for board input"
        )
    else:
        detail = str(state_payload.get("reason") or "").replace("_", " ").capitalize()
    return headline, detail, tone


def teleop_backend_text(state_payload):
    """Which command gate is open, as reported by the adapter."""
    if not isinstance(state_payload, dict):
        return ""
    if state_payload.get("hardware_active") is True:
        return "Hardware command gate open"
    if state_payload.get("simulation_commands_enabled") is True:
        return "Simulation commands · hardware inactive"
    return "Commands disabled"
