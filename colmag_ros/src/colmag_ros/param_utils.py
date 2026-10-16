"""ROS-free parameter coercion helpers shared by COLMAG nodes."""

from __future__ import annotations

import math
from typing import Any, Optional


def bool_param(value: Any) -> bool:
    """Parse a ROS-style boolean; strings like ``"true"``/``"on"``/``"1"`` are true."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def finite_float(value: Any, default: float = 0.0) -> float:
    """Return ``value`` as a finite float, or ``default`` when unparsable or non-finite."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def optional_float(value: Any) -> Optional[float]:
    """Return ``value`` as a float, or ``None`` for ``None``, empty/``none``/``nan`` text, or unparsable input."""
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("", "none", "nan"):
            return None
        try:
            return float(text)
        except ValueError:
            return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
