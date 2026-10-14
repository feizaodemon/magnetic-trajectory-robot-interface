"""OCR canvas mode for the trajectory dashboard."""

import json
import time

from colmag_ros import dashboard_points as _dashboard_points
from colmag_ros.dashboard_confirm_publisher import (
    build_mouse_toolbar_stop_payload,
    should_publish_mouse_toolbar_stop,
)
from colmag_ros.dashboard_controller_input import cancelled_preview_hover_state
from colmag_ros.dashboard_geometry import preview_button_style
from colmag_ros.dashboard_widgets import draw_control_button


class DashboardOcrPanelMixin:
    """Handles OCR canvas dwell, toolbar hover and actions, and OCR recognition timing."""

    def _process_ocr_toolbar_dwell(self):
        if self.config.demo_input_mode == "ocr_canvas":
            self._process_ocr_canvas_dwell()
            self.root.after(50, self._process_ocr_toolbar_dwell)
            return
        if not self._preview_dwell_active():
            return
        self._process_preview_dwell()
        self.root.after(50, self._process_ocr_toolbar_dwell)

    def _process_ocr_canvas_dwell(self):
        """OCR-canvas hover-dwell (mouse only). Unchanged legacy behaviour."""
        now = time.time()
        timer = self.dwell_timer
        button_id = timer.button
        enabled = bool(button_id) and self._ocr_button_enabled(button_id)
        self.hover_source = "mouse" if button_id else "none"
        if not enabled:
            # Unlike the preview loop, a disabled button restarts its dwell.
            timer.start(button_id, now)
        elif timer.advance(now, self.config.dwell_confirm_sec):
            self._handle_ocr_action(button_id)
        self.dwell_state = timer.dwell_state(enabled, report_activation=False)
        self._refresh_dwell_status()
        self._update_ocr_toolbar_visuals()
        self._publish_dashboard_state()

    def _ocr_button_enabled(self, button_id):
        """Mouse buttons are off in TELEOP; C also needs a result to confirm."""
        if self._teleop_active():
            return False
        return button_id != "C" or self._ocr_confirm_enabled()

    def _set_ocr_hover_button(self, button_id):
        button_id = str(button_id or "").upper()
        if self.config.demo_input_mode != "ocr_canvas":
            # Preview mode: record the raw MOUSE hover only. The dwell loop decides
            # the effective button (board cursor first, this mouse hover as
            # fallback), so mouse dwell timing is owned by the loop, not here.
            self.mouse_hover_button = button_id
            return
        if button_id == self.dwell_timer.button:
            return
        self.dwell_timer.start(button_id, time.time())
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()

    def _clear_ocr_hover_button(self, button_id=None):
        if self.config.demo_input_mode != "ocr_canvas":
            if button_id is not None and str(button_id or "").upper() != self.mouse_hover_button:
                return
            self.mouse_hover_button = ""
            return
        if button_id is not None and str(button_id or "").upper() != self.dwell_timer.button:
            return
        self.dwell_timer.reset()
        self.dwell_state = cancelled_preview_hover_state()
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()

    def _update_ocr_toolbar_visuals(self):
        dwell_buttons = self.config.demo_input_mode == "ocr_canvas" or (
            self.config.is_trajectory_mode and self.config.candidate_preview_enabled)
        for button_id, widgets in self.ocr_button_widgets.items():
            canvas = widgets["canvas"]
            size = widgets.get("size", 82)
            progress = 0.0
            if dwell_buttons:
                if self.config.demo_input_mode == "ocr_canvas":
                    enabled = self._ocr_button_enabled(button_id)
                else:
                    enabled = self._preview_button_enabled(button_id)
                active = button_id == self.dwell_timer.button
                style = preview_button_style(button_id, enabled, active=active)
                if active and enabled and self.config.hover_progress_enabled:
                    progress = self.dwell_timer.progress
            else:
                style = preview_button_style(button_id, True)

            canvas.delete("all")
            draw_control_button(canvas, size / 2.0, size / 2.0, size / 2.0 - 2,
                                widgets["base_text"], style, progress)

    def _handle_ocr_action(self, button_id):
        button_id = str(button_id or "").upper()
        if self.config.demo_input_mode == "ocr_canvas" and self._teleop_active():
            return  # Mouse actions are paused while TELEOP owns the robot.
        if button_id == "B":
            if should_publish_mouse_toolbar_stop(self.ocr_panel_state):
                self._start_ocr_drawing("Drawing mode started")
        elif button_id == "C":
            if not self._ocr_confirm_enabled():
                return
            self._cancel_ocr_auto_recognize()
            self._cancel_ocr_recognition_timeout()
            self._confirm_rank_by_dwell(1)
            self.ocr_panel_state = "WAITING"
            self.all_strokes = []
            self.drawing_stroke = []
            self.root.after(0, lambda: self.ui_state_var.set("State: WAITING (Confirmed)"))
        elif button_id == "X":
            self._cancel_ocr_auto_recognize()
            self._cancel_ocr_recognition_timeout()
            self.ocr_panel_state = "WAITING"
            self.captured_path = []
            self.all_strokes = []
            self.drawing_stroke = []
            self._clear_candidates("Cleared canvas")
            self.root.after(0, lambda: self.ui_state_var.set("State: WAITING (Cleared)"))
        elif button_id == "A":
            if self.ocr_panel_state != "BLOCKED":
                self._cancel_ocr_auto_recognize()
                self._cancel_ocr_recognition_timeout()
                self.ocr_panel_state = "BLOCKED"
                self._publish_ocr_stop_label()
            self.root.after(0, lambda: self.ui_state_var.set("State: BLOCKED (STOP)"))
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

    def _start_ocr_drawing(self, message):
        self._cancel_ocr_auto_recognize()
        self._cancel_ocr_recognition_timeout()
        self.ocr_panel_state = "DRAWING"
        self.captured_path = []
        self.all_strokes = []
        self.drawing_stroke = []
        self.ocr_last_stroke_time = None
        self._clear_candidates(message)
        self.rospy.loginfo("ocr drawing started state=%s", self.ocr_panel_state)
        self.root.after(0, lambda: self.ui_state_var.set("State: DRAWING"))
        self.root.after(0, lambda: self.capture_var.set("Draw a symbol, then release"))

    def _collect_ocr_points(self):
        return _dashboard_points.flatten_strokes(self.all_strokes)

    def _cancel_ocr_auto_recognize(self):
        if self.ocr_auto_recognize_job is None:
            return
        try:
            self.root.after_cancel(self.ocr_auto_recognize_job)
        except self.tk.TclError:
            pass
        self.ocr_auto_recognize_job = None

    def _cancel_ocr_recognition_timeout(self):
        if self.ocr_recognition_timeout_job is None:
            return
        try:
            self.root.after_cancel(self.ocr_recognition_timeout_job)
        except self.tk.TclError:
            pass
        self.ocr_recognition_timeout_job = None

    def _schedule_ocr_auto_recognize(self):
        self._cancel_ocr_auto_recognize()
        points = self._collect_ocr_points()
        if len(points) < 2:
            self.root.after(0, lambda: self.capture_var.set("Draw a symbol before recognizing"))
            return
        self.ocr_last_stroke_time = time.time()
        self.ocr_auto_recognize_job = self.root.after(
            self.ocr_auto_recognize_after_ms,
            self._trigger_ocr_recognition_if_ready,
        )
        self.root.after(0, lambda: self.capture_var.set("Recognition will start automatically"))

    def _trigger_ocr_recognition_if_ready(self):
        self.ocr_auto_recognize_job = None
        if self.ocr_panel_state != "DRAWING":
            return
        points = self._collect_ocr_points()
        if len(points) < 2:
            self.root.after(0, lambda: self.capture_var.set("Draw a symbol before recognizing"))
            return
        self.ocr_panel_state = "RECOGNIZING"
        payload_points = [[x, y] for x, y in points]
        payload = {
            "points": payload_points,
            "sample_id": "ocr_canvas_%d" % int(time.time()),
            "sequence_id": 0,
        }
        self.symbol_capture_pub.publish(self.String(data=json.dumps(payload)))
        self.rospy.loginfo(
            "ocr capture published sample_id=%s point_count=%d state=%s",
            payload["sample_id"],
            len(payload_points),
            self.ocr_panel_state,
        )
        self._cancel_ocr_recognition_timeout()
        self.ocr_recognition_timeout_job = self.root.after(
            self.config.ocr_recognition_timeout_ms,
            self._handle_ocr_recognition_timeout,
        )
        self.root.after(0, lambda: self.ui_state_var.set("State: RECOGNIZING"))
        self.root.after(0, lambda: self.capture_var.set(
            f"Sent {len(points)} points for recognition"))
        self._update_ocr_toolbar_visuals()
        self._refresh_dwell_status()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

    def _handle_ocr_recognition_timeout(self):
        self.ocr_recognition_timeout_job = None
        if self.ocr_panel_state != "RECOGNIZING":
            return
        self.ocr_panel_state = "DRAWING"
        self.rospy.logwarn("ocr recognition timeout: no candidates received")
        self.root.after(0, lambda: self.ui_state_var.set("State: DRAWING (No recognizer response)"))
        self.root.after(0, lambda: self.capture_var.set("No recognizer response"))
        self._update_ocr_toolbar_visuals()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

    def _publish_ocr_stop_label(self):
        if self.confirm_pub is None or not self.config.integrated_confirm_enabled:
            return
        confirmed = build_mouse_toolbar_stop_payload(time.time())
        self.last_confirmed_label = "X"
        self.last_selected_rank = None
        self.last_command_intent = "STOP_OR_CANCEL"
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Mouse toolbar: A -> STOP_OR_CANCEL"))
        self._publish_dashboard_state()
        self.rospy.loginfo("mouse toolbar STOP_OR_CANCEL confirmed_label=%s", msg.data)
