"""Current ROS JSON adapters for the serial and mock packet sources."""

from __future__ import annotations

import json
import sys
from pathlib import Path


def ensure_project_src_path(project_src_path: str) -> None:
    """Make the repository's ``serial_mvp`` package importable."""
    if project_src_path is None or not str(project_src_path).strip():
        raise ValueError(
            "missing_project_src_path: direct invocation requires "
            "_project_src_path:=<path-to-repository-src>"
        )

    root = Path(str(project_src_path).strip()).expanduser()
    if (root / "serial_mvp").is_dir():
        source_root = root
    elif (root / "src" / "serial_mvp").is_dir():
        source_root = root / "src"
    else:
        raise ValueError(
            "invalid_project_src_path: expected a directory containing serial_mvp; "
            "provide _project_src_path:=<path-to-repository-src>"
        )

    path = str(source_root)
    if path not in sys.path:
        sys.path.insert(0, path)


def make_mock_raw_packet_json(timestamp: float, x: float, y: float, z: float = 0.0) -> str:
    """Build the raw-packet shape consumed by the current trajectory bridge."""
    readings = [{"bx": 0.0, "by": 0.0, "bz": 0.0} for _ in range(15)]
    readings.append({"bx": 6000.0, "by": 0.0, "bz": 0.0})
    payload = {
        "timestamp": timestamp,
        "magnetic_readings": readings,
        "tracking_outputs": [x, y, z, 0.0, 0.0, 1.0],
    }
    return json.dumps(payload, separators=(",", ":"))


def packet_to_raw_json(timestamp: float, packet_data) -> str:
    """Serialize one decoded serial packet to the current raw-packet shape."""
    payload = {
        "timestamp": timestamp,
        "magnetic_readings": [
            {"bx": reading.bx, "by": reading.by, "bz": reading.bz}
            for reading in packet_data.magnetic_readings
        ],
        "tracking_outputs": list(packet_data.tracking_outputs),
    }
    return json.dumps(payload, separators=(",", ":"))
