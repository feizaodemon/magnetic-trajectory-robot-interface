"""Repository paths shared by the offline scripts in this folder.

Importing it puts ``colmag_ros/src`` on ``sys.path`` so a script run as
``python3 scripts/<name>.py`` can import the ROS-free ``colmag_ros`` package
without a catkin workspace. It starts nothing and writes nothing.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO / "colmag_ros" / "src"
# The tracked 8-symbol DTW bank. The launch files name the same file in their
# own arg defaults; those stay as they are.
TRACKED_BANK = REPO / "colmag_ros" / "config" / "dtw_banks" / "m104c1_8symbol_mouse_seeded_yflip.json"

if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))
