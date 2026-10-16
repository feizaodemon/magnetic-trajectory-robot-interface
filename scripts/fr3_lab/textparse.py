"""Small text, hashing and time helpers with no I/O.

Moved unchanged from scripts/fr3_lab_runtime.py, which re-exports every
name. Real-FR3 lab tooling: keep values and behaviour as they are.
"""

from __future__ import annotations

import ast
import hashlib
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple


def parse_sequence(text: str) -> List[object]:
    stripped = text.strip()
    if not stripped:
        return []
    if stripped.startswith("["):
        try:
            value = ast.literal_eval(stripped)
            return list(value) if isinstance(value, (list, tuple)) else []
        except (SyntaxError, ValueError):
            return [item.strip().strip('"\'') for item in stripped.strip("[]").split(",") if item.strip()]
    return [line.strip()[1:].strip().strip('"\'') for line in text.splitlines() if line.strip().startswith("-")]


def parse_named_sequence(text: str, key: str) -> List[object]:
    lines = text.splitlines()
    for index, raw in enumerate(lines):
        if raw.strip().startswith(key + ":"):
            inline = raw.split(":", 1)[1].strip()
            if inline:
                return parse_sequence(inline)
            collected: List[str] = []
            for following in lines[index + 1 :]:
                stripped = following.strip()
                if stripped.startswith("-"):
                    collected.append(stripped)
                    continue
                if stripped:
                    break
            return parse_sequence("\n".join(collected))
    return []


def redact(text: str, secrets: Iterable[str]) -> str:
    result = text
    for secret in secrets:
        if secret:
            result = result.replace(secret, "<redacted>")
    result = re.sub(r"(?i)(FR3_ROBOT_IP\s*[:=]\s*)\S+", r"\1<redacted>", result)
    result = re.sub(
        r'(?i)(adapter_session_id["\']?\s*[:=]\s*["\']?)[^"\',\s}]+',
        r"\1<redacted>",
        result,
    )
    return result


def command_fingerprint(args: Sequence[str], secrets: Iterable[str]) -> str:
    safe = [redact(str(arg), secrets) for arg in args]
    return sha256_text("\0".join(safe))


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def parse_utc(value: str) -> Optional[datetime]:
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return _utc(parsed)


def _utc(value: datetime) -> datetime:
    if value.tzinfo is None:
        return value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def _text_or_empty(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _ros_bool(value: bool) -> str:
    return "true" if value else "false"


def _unquote(value: str) -> str:
    return value.strip().strip('"\'')


def _string_tuple(value: object) -> Tuple[str, ...]:
    if not isinstance(value, list):
        return ()
    return tuple(str(item) for item in value)


def _int_value(value: object, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _parse_scalar(value: str) -> object:
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _yes_no(value: object) -> str:
    if value is None:
        return "no"
    return "yes" if bool(value) else "no"


def _enabled_value(value: object) -> str:
    if value is True:
        return "enabled"
    if value is False:
        return "disabled"
    return "unknown"


def _goal_count_value(state: Mapping[str, object]) -> str:
    if "motion_goal_send_attempts" not in state:
        return "unknown"
    return "{}/1".format(_int_value(state.get("motion_goal_send_attempts"), 0))
