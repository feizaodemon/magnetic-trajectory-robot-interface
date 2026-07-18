# -*- coding: utf-8 -*-

"""Preview-only trajectory segment recorder (M63-B).

A tiny, dependency-free state machine that captures ONE complete trajectory
segment after a Start trigger, instead of recognizing an uncontrolled continuous
stream. It never touches ROS, Tk, numpy, EasyOCR, a robot, a task command, or a
confirmed label. It only decides *which* points belong to the current segment and
*when* that segment is complete, so a downstream recognizer/preview can run on a
bounded, complete stroke.

Capture states owned by this recorder:

    IDLE            -> nothing recorded yet / reset
    ARMED           -> Start pressed, waiting for the first writing point
    RECORDING       -> collecting points for the current segment
    RECORDING_DONE  -> a complete segment is ready to recognize (preview only)
    CLEARED         -> the segment was explicitly cleared

The downstream preview UI owns the later states (RECOGNIZING, SHOWING_CANDIDATES,
PREVIEW_CONFIRMED); they are exported here only as shared string constants so the
dashboard and tests use one vocabulary.

Stop conditions (whichever happens first):

  * time window: ``now - start_time >= duration_sec`` (primary, always available)
  * lift-away:   ``stop_on_lift`` and the sample ``z`` exceeds
                 ``lift_z_threshold`` (only when the trajectory schema exposes z;
                 the caller passes ``z=None`` when z is unavailable and the
                 lift-stop is simply skipped)
  * hard cap:    ``len(points) >= max_points`` (safety bound on memory)

The window is measured from the FIRST recorded point, not from Start, so a short
arming delay before the pen touches down does not shorten the segment.
"""

# Capture states (owned by this recorder).
STATE_IDLE = "IDLE"
STATE_ARMED = "ARMED"
STATE_RECORDING = "RECORDING"
STATE_RECORDING_DONE = "RECORDING_DONE"
STATE_CLEARED = "CLEARED"

# Downstream preview states (owned by the UI; shared here for one vocabulary).
STATE_RECOGNIZING = "RECOGNIZING"
STATE_SHOWING_CANDIDATES = "SHOWING_CANDIDATES"
STATE_PREVIEW_CONFIRMED = "PREVIEW_CONFIRMED"

CAPTURE_STATES = (
    STATE_IDLE,
    STATE_ARMED,
    STATE_RECORDING,
    STATE_RECORDING_DONE,
    STATE_CLEARED,
)


def _num(value):
    """Return a finite float or None (rejects bool, NaN, inf, non-numbers)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    f = float(value)
    if f != f or f in (float("inf"), float("-inf")):
        return None
    return f


class SegmentRecorder:
    """Capture one complete trajectory segment after a Start trigger.

    Pure and side-effect free: no ROS, Tk, threads, or timers. The owner feeds
    samples and (optionally) polls for the time-window stop; the recorder only
    tracks state and the collected points.
    """

    def __init__(self, duration_sec=2.5, stop_on_lift=False,
                 lift_z_threshold=0.05, min_points=8, max_points=600):
        self.duration_sec = max(0.1, float(duration_sec))
        self.stop_on_lift = bool(stop_on_lift)
        self.lift_z_threshold = float(lift_z_threshold)
        self.min_points = max(1, int(min_points))
        self.max_points = max(self.min_points, int(max_points))
        self.state = STATE_IDLE
        self.points = []
        self.start_time = None
        self.stop_reason = ""

    # --- state queries -------------------------------------------------------
    @property
    def is_recording(self):
        return self.state == STATE_RECORDING

    @property
    def is_armed(self):
        return self.state == STATE_ARMED

    @property
    def is_done(self):
        return self.state == STATE_RECORDING_DONE

    @property
    def has_enough_points(self):
        return len(self.points) >= self.min_points

    def elapsed(self, now):
        if self.start_time is None:
            return 0.0
        return max(0.0, float(now) - self.start_time)

    def segment(self):
        """Return a copy of the recorded segment points (oldest -> newest)."""
        return list(self.points)

    # --- control -------------------------------------------------------------
    def start(self, now):
        """Start a new segment: clear the previous one and arm capture.

        Recording begins on the first writing point fed after this call, so the
        time window is measured from real pen-down, not from the button press.
        """
        self.points = []
        self.start_time = None
        self.stop_reason = ""
        self.state = STATE_ARMED
        return self.state

    def clear(self):
        """Discard any segment and mark CLEARED (preview only)."""
        self.points = []
        self.start_time = None
        self.stop_reason = "cleared"
        self.state = STATE_CLEARED
        return self.state

    def reset(self):
        """Return to IDLE (no segment, no reason)."""
        self.points = []
        self.start_time = None
        self.stop_reason = ""
        self.state = STATE_IDLE
        return self.state

    # --- capture -------------------------------------------------------------
    def feed(self, x, y, now, z=None, valid=True, writing=True):
        """Feed one live sample. Returns the current state.

        Only valid, writing points are recorded. Invalid / non-writing points do
        not extend the segment but still allow the time-window and lift stops to
        fire. Does nothing outside ARMED / RECORDING.
        """
        if self.state not in (STATE_ARMED, STATE_RECORDING):
            return self.state

        now = float(now)
        z_val = _num(z)
        recordable = bool(valid) and bool(writing)

        if self.state == STATE_ARMED:
            if not recordable:
                return self.state
            self.state = STATE_RECORDING
            self.start_time = now
            self.points = []

        # Lift-away stop (only when z is available and enabled).
        if (self.stop_on_lift and z_val is not None
                and z_val > self.lift_z_threshold):
            return self._finish("lift", now)

        if recordable:
            self.points.append((float(x), float(y)))
            if len(self.points) >= self.max_points:
                return self._finish("max_points", now)

        if now - self.start_time >= self.duration_sec:
            return self._finish("duration", now)
        return self.state

    def poll(self, now):
        """Time-window check without a new sample. Returns the current state.

        Lets the owner finalize a segment when live samples pause exactly at the
        end of the window. No-op outside RECORDING.
        """
        if self.state != STATE_RECORDING or self.start_time is None:
            return self.state
        if float(now) - self.start_time >= self.duration_sec:
            return self._finish("duration", now)
        return self.state

    def finish_now(self, now, reason="manual"):
        """Force-finalize the current recording (e.g. on Recognize).

        Returns the current state; only acts while RECORDING.
        """
        if self.state != STATE_RECORDING:
            return self.state
        return self._finish(reason, now)

    def _finish(self, reason, now):
        self.stop_reason = reason
        self.state = STATE_RECORDING_DONE
        return self.state
