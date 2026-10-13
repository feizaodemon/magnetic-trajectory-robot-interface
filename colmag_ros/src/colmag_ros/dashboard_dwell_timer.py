"""Hover-dwell timing for the dashboard's virtual buttons.

A ``DwellTimer`` tracks the hovered button, when dwell on it started, the
progress toward activation, and whether the current entry already fired. It
has no Tk or ROS dependency; the dashboard's dwell loops tick it.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class DwellTimer:
    button: str = ""
    started_at: Optional[float] = None
    progress: float = 0.0
    fired: bool = False

    def start(self, button, now):
        """Hover ``button`` ("" for none) and restart its dwell from ``now``."""
        self.button = button
        self.started_at = now if button else None
        self.progress = 0.0
        self.fired = False

    def reset(self):
        self.start("", None)

    def hold(self):
        """The hovered button is disabled: show no progress, keep the entry time."""
        self.progress = 0.0

    def advance(self, now, dwell_sec):
        """Accumulate dwell on an enabled button.

        Returns True exactly once per entry, on the tick that completes the dwell.
        """
        if self.started_at is None:
            self.started_at = now
        self.progress = max(0.0, min(1.0, (now - self.started_at) / max(dwell_sec, 1e-9)))
        if self.progress >= 1.0 and not self.fired:
            self.fired = True
            return True
        return False

    def dwell_state(self, enabled, report_activation=True):
        """The ``dwell_state`` dict the dashboard status and state topic read."""
        return {
            "current_button_id": self.button if enabled else "",
            "started_at": self.started_at,
            "dwell_progress": self.progress if enabled else 0.0,
            "activated_button_id": (
                self.button if (enabled and self.fired and report_activation) else ""),
        }
