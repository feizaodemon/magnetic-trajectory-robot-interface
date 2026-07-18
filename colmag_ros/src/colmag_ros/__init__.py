"""ROS1 wrapper helpers for the COLMAG serial MVP."""

from pathlib import Path


# Keep legacy ``colmag_ros.scripts`` source-tree imports available while the
# canonical helper modules live in this installable package.
_SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if (_SOURCE_PACKAGE_ROOT / "scripts").is_dir():
    __path__.append(str(_SOURCE_PACKAGE_ROOT))
