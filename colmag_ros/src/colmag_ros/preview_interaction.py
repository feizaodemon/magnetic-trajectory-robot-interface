"""Trajectory-preview interaction state of the dashboard.

``PreviewInteraction`` owns three values the operator panel shows:

- ``state``: where the operator is in draw -> recognize -> confirm.
- ``candidate_status``: the recognizer status of the current sample.
- ``confirmed_label``: the label confirmed in the preview, if any.

Each transition is a named method, so the state vocabulary lives here. It has
no Tk or ROS dependency; the dashboard mixins call it and do the UI updates.
"""


class PreviewInteraction:
    def __init__(self):
        self.state = "idle"
        self.candidate_status = "idle"
        self.confirmed_label = ""

    def _set(self, candidate_status, state):
        self.candidate_status = candidate_status
        self.state = state

    # Operator actions.
    def clear(self):
        self._set("idle", "cleared")
        self.confirmed_label = ""

    def reject(self):
        self._set("rejected", "rejected")
        self.confirmed_label = ""

    def confirm(self, label, has_candidates):
        """Confirm ``label``; without a label, fall back to ready/idle."""
        if not label:
            self.state = "ready" if has_candidates else "idle"
            return False
        self.confirmed_label = label
        self.state = "preview_confirmed"
        return True

    # Drawing and capture.
    def new_drawing(self):
        self._set("idle", "new_drawing_started")

    def too_few_points(self):
        self._set("collecting", "too_few_drawing_points")

    def capture_published(self):
        self._set("recognizing", "symbol_capture_published")

    # Segment recording.
    def recording(self):
        self._set("recording", "recording")

    def recording_done(self):
        self._set("recording_done", "recording_done")

    # In-process recognition.
    def recognizing(self):
        self._set("recognizing", "recognizing")

    def recognized(self, status):
        """Record a ``recognize_stroke`` status and derive the interaction state."""
        if status == "ready":
            state = "candidates_ready"
        elif status == "collecting":
            state = "collecting"
        elif status in ("unavailable", "error"):
            state = "error"
        else:
            state = status
        self._set(status, state)

    # External /colmag/symbol_candidates.
    def external_candidates(self, has_candidates, uncertain):
        if has_candidates:
            self._set("ready", "external_candidates_ready")
        elif uncertain:
            self._set("uncertain", "external_candidate_uncertain")
        else:
            self._set("none", "external_no_candidate")
