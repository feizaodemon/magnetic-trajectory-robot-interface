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

GRIPPER_KEYS = {"o": "open", "c": "close"}

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


def gripper_command_for_key(key):
    """Return ``"open"`` / ``"close"`` for a gripper key, or ``None``."""
    key = str(key or "")
    return GRIPPER_KEYS.get(key.lower()) if len(key) == 1 else None


def build_gripper_payload(command, sequence_id, timestamp=None):
    """Build one discrete gripper command for the Gazebo gripper bridge."""
    if command not in GRIPPER_KEYS.values():
        raise ValueError("unknown gripper command: %r" % command)
    timestamp = time.time() if timestamp is None else float(timestamp)
    return {
        "timestamp": timestamp,
        "sequence_id": int(sequence_id),
        "command": command,
        "input_mode": "keyboard_gripper",
    }


def build_release_payload(sequence_id, timestamp=None):
    """Build an engaged zero-direction input: the held key was released."""
    timestamp = time.time() if timestamp is None else float(timestamp)
    return {
        "timestamp": timestamp,
        "sequence_id": int(sequence_id),
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "engaged": True,
        "input_mode": "keyboard_cartesian",
    }


HOLD = "hold"
RELEASE = "release"


class KeyHoldTracker:
    """Turns terminal auto-repeat into a held direction key.

    A terminal reports a held key as one press, a pause of about 0.5 s, then
    repeats every 30-40 ms. It never reports the release. The tracker keeps a
    new key held through the first-repeat pause and treats a missing repeat
    as a release.
    """

    def __init__(self, first_repeat_window_sec=0.6, repeat_window_sec=0.12):
        self.first_repeat_window_sec = float(first_repeat_window_sec)
        self.repeat_window_sec = float(repeat_window_sec)
        self.key = None
        self.deadline = 0.0

    def press(self, key, now):
        """Register one direction key event; ``True`` when it starts a new hold."""
        if key == self.key:
            self.deadline = max(self.deadline, now + self.repeat_window_sec)
            return False
        self.key = key
        self.deadline = now + max(self.first_repeat_window_sec, self.repeat_window_sec)
        return True

    def clear(self):
        self.key = None

    def poll(self, now):
        """``HOLD`` while the key counts as held, ``RELEASE`` once, else ``None``."""
        if self.key is None:
            return None
        if now < self.deadline:
            return HOLD
        self.key = None
        return RELEASE


def help_lines():
    return (
        "W / Up -> +X; S / Down -> -X",
        "A / Left -> +Y; D / Right -> -Y",
        "Q -> +Z; E -> -Z",
        "O -> open gripper; C -> close gripper (Gazebo gripper bridge)",
        "SPACE -> pause and invalidate target; ESC / Ctrl-C -> quit",
        "Hold a key to move; releasing it brakes the robot to rest.",
    )
