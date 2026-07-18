#!/usr/bin/env python3
"""Compatibility wrapper for :mod:`colmag_ros.m104c2c3_display_semantics`."""

try:
    from _colmag_package_compat import ensure_colmag_ros_package
except ImportError:
    from ._colmag_package_compat import ensure_colmag_ros_package

ensure_colmag_ros_package()
from colmag_ros.m104c2c3_display_semantics import *  # noqa: F401,F403
