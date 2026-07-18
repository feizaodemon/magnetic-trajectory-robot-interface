"""ROS-free helpers shared by COLMAG Gazebo stub nodes."""

from pathlib import Path


_SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if (_SOURCE_PACKAGE_ROOT / "scripts").is_dir():
    __path__.append(str(_SOURCE_PACKAGE_ROOT))
