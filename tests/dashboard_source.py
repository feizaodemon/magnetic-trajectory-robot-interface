"""Source text of the dashboard node together with its package mixin modules.

Static tests that assert on dashboard method text read it from here, so they do
not depend on whether a method lives in the node or in one of its mixins.
"""

import re
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
PACKAGE = REPO / "colmag_ros" / "src" / "colmag_ros"
NODE_FILE = REPO / "colmag_ros" / "scripts" / "magnetic_trajectory_dashboard_node.py"
MIXIN_IMPORT = re.compile(r"^from colmag_ros\.(\w+) import \w+Mixin$", re.MULTILINE)


def dashboard_files():
    """Return the node file followed by every mixin module it imports."""
    modules = MIXIN_IMPORT.findall(NODE_FILE.read_text())
    assert modules, "dashboard node imports no mixins; update MIXIN_IMPORT"
    return [NODE_FILE] + [PACKAGE / (name + ".py") for name in modules]


def dashboard_source():
    return "\n".join(path.read_text() for path in dashboard_files())
