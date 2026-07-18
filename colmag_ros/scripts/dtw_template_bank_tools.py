#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`colmag_ros.dtw_template_bank_tools`."""

try:
    from _colmag_package_compat import ensure_colmag_ros_package
except ImportError:
    from ._colmag_package_compat import ensure_colmag_ros_package

ensure_colmag_ros_package()
from colmag_ros.dtw_template_bank_tools import *  # noqa: F401,F403
