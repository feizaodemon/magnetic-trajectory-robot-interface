"""Source-tree compatibility for legacy imports from ``colmag_ros/scripts``."""

import sys
from pathlib import Path


def ensure_colmag_ros_package():
    source_root = Path(__file__).resolve().parents[1] / "src"
    source_text = str(source_root)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)
