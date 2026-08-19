"""ROS-free key mapping for continuous Cartesian development input."""

import time


KEY_DIRECTIONS = {
    "w": (1.0, 0.0, 0.0),
    "\x1b[A": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "\x1b[B": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "\x1b[D": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "\x1b[C": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
}

PAUSE_KEYS = frozenset((" ",))
QUIT_KEYS = frozenset(("\x03", "\x1b"))
HELP_KEYS = frozenset(("?",))


def direction_for_key(key):
    """Return one normalized world-frame Cartesian direction, or ``None``."""
    key = str(key or "")
    return KEY_DIRECTIONS.get(key.lower() if len(key) == 1 else key)


def build_cartesian_input_payload(key, sequence_id, timestamp=None):
    """Build one bounded input pulse for the shared Cartesian core adapter."""
    direction = direction_for_key(key)
    if direction is None and key not in PAUSE_KEYS:
        raise ValueError("unknown Cartesian teleop key: %r" % key)
    timestamp = time.time() if timestamp is None else float(timestamp)
    x, y, z = direction or (0.0, 0.0, 0.0)
    return {
        "timestamp": timestamp,
        "sequence_id": int(sequence_id),
        "x": x,
        "y": y,
        "z": z,
        "engaged": direction is not None,
        "input_mode": "keyboard_cartesian",
    }


def help_lines():
    return (
        "W / Up -> +X; S / Down -> -X",
        "A / Left -> +Y; D / Right -> -Y",
        "Q -> +Z; E -> -Z",
        "SPACE -> pause and invalidate target; ESC / Ctrl-C -> quit",
        "Hold a key for continuous auto-repeat input; stale input pauses motion.",
    )
