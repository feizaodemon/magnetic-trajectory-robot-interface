"""ROS-free command mapping for the optional task-level keyboard source."""

import time

from colmag_ros import m104c4_execution_semantics


COMMAND_ALIASES = {
    "left": "1",
    "l": "1",
    "1": "1",
    "hex": "2",
    "hexagon": "2",
    "2": "2",
    "right": "3",
    "r": "3",
    "3": "3",
    "hover": "V",
    "up": "V",
    "v": "V",
    "orbit": "O",
    "o": "O",
    "stop": "X",
    "cancel": "X",
    "x": "X",
    "home": "A",
    "ready": "A",
    "a": "A",
    "down": "C",
    "descend": "C",
    "c": "C",
}

QUIT_COMMANDS = frozenset(("q", "quit", "exit"))
HELP_COMMANDS = frozenset(("?", "help"))


def normalize_command(command):
    """Return a C4 label for one terminal command, or ``None`` if unknown."""
    return COMMAND_ALIASES.get(str(command or "").strip().lower())


def task_for_command(command):
    """Resolve a terminal command through the canonical semantic task contract."""
    label = normalize_command(command)
    semantics = m104c4_execution_semantics.execution_semantics_for_label(label)
    return semantics["task"] if semantics else None


def build_confirmed_label_payload(command, sequence_id, timestamp=None):
    """Build the manual-confirm payload consumed by ``task_dispatcher_node``."""
    label = normalize_command(command)
    if label is None:
        raise ValueError("unknown keyboard teleop command: %r" % command)

    timestamp = time.time() if timestamp is None else float(timestamp)
    return {
        "timestamp": timestamp,
        "sample_id": "keyboard-teleop-%d" % int(sequence_id),
        "sequence_id": int(sequence_id),
        "label": label,
        "confidence": 1.0,
        "confirmed": True,
        "confirmed_by": "terminal_keyboard_teleop",
        "input_mode": "terminal_keyboard",
    }


def help_lines():
    """Return stable, operator-facing command help."""
    return (
        "left/l/1 -> MOVE_LEFT",
        "hex/2 -> HEXAGON_TRAJECTORY",
        "right/r/3 -> MOVE_RIGHT",
        "hover/up/v -> HOVER_APPROACH",
        "orbit/o -> ORBIT_SMALL",
        "stop/cancel/x -> STOP_OR_CANCEL",
        "home/ready/a -> HOME_OR_READY",
        "down/descend/c -> SOFT_DESCEND_PREVIEW",
        "help/? -> show controls; quit/q -> exit",
    )
