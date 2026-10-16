"""Small, ROS-free contract for the two operator control modes."""

from __future__ import annotations

from typing import Any, Optional

TASK = "TASK"
TELEOP = "TELEOP"
SUPPORTED_MODES = frozenset((TASK, TELEOP))


def normalize_mode(value: Any) -> Optional[str]:
    """Return a canonical mode, or ``None`` for an unsupported value."""
    mode = str(value or "").strip().upper()
    return mode if mode in SUPPORTED_MODES else None


def startup_mode(value: Any = None) -> str:
    """Fail closed to TASK when an initial mode is absent or invalid."""
    return normalize_mode(value) or TASK


def mode_allows(required_mode: Any, current_mode: Any, state_observed: Any = True) -> bool:
    """Return whether a mode-controlled command route may admit work."""
    required = normalize_mode(required_mode)
    if required is None:
        return not str(required_mode or "").strip()
    return bool(state_observed) and normalize_mode(current_mode) == required
