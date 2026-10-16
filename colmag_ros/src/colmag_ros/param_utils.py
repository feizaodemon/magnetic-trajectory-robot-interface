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


def strict_bool_param(value: Any, name: str) -> bool:
    """Parse a boolean param, raising ``ValueError`` unless it is clearly true or false.

    Accepts bools, the integers 0 and 1, and the strings ``true``/``false``,
    ``yes``/``no``, ``on``/``off`` and ``1``/``0`` (trimmed, any case). Unlike
    ``bool(value)``, the string ``"false"`` is False; unlike ``bool_param``, a
    misspelled value fails loudly instead of reading as False.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "on"):
            return True
        if text in ("0", "false", "no", "off"):
            return False
    raise ValueError("%s must be true or false, got %r" % (name, value))


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
